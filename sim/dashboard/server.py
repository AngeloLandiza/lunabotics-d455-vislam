#!/usr/bin/env python3
"""Local dashboard for the BP-1 sandbox: machine load, run progress, results, parameters and research checks.

    python3 sim/dashboard/server.py [--port 8765]

then open http://127.0.0.1:8765/ in a browser. Standard library only (Python 3.10+); binds to 127.0.0.1.
Every /api/* endpoint returns JSON; on any failure it returns {"error": ...} with HTTP 500. See README.md.
"""
import argparse
import importlib
import json
import math
import os
import re
import shutil
import socket
import subprocess
import sys
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlparse

sys.dont_write_bytecode = True   # importing params.py must not leave .pyc files behind

DASH = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(DASH))
SANDBOX = os.path.join(REPO, "sim", "bp1_sandbox")
SUMMARIES = os.path.join(SANDBOX, "summaries")
FIDUCIAL_RESULTS = os.path.join(REPO, "sim", "fiducial", "results")
ODOMETRY_RESULTS = os.path.join(REPO, "sim", "odometry", "results")
AUTONOMY_RUNS = os.path.join(REPO, "sim", "autonomy_runs")
RESEARCH_INDEX = os.path.join(DASH, "research_index.json")
STATIC = {"/": ("index.html", "text/html; charset=utf-8"),
          "/index.html": ("index.html", "text/html; charset=utf-8"),
          "/app.js": ("app.js", "text/javascript; charset=utf-8"),
          "/style.css": ("style.css", "text/css; charset=utf-8")}
STALE_S = 600            # a run with no line for this long and no end line is stale
RESULT_DEPTH = 3         # directory levels searched below each results root
PROC_WORDS = ("approach.py", "robustness.py", "speed_gate.py", "evaluate.py", "isaac", "live_view")
KIT_RE = re.compile(r"(^|[/\s])kit($|[/\s.])")
RUN_ID_RE = re.compile(r"^[A-Za-z0-9._-]{1,200}$")
HOST = socket.gethostname()
CLK_TCK = os.sysconf("SC_CLK_TCK") if hasattr(os, "sysconf") else 100
PAGE = os.sysconf("SC_PAGE_SIZE") if hasattr(os, "sysconf") else 4096


def telemetry_dir():
    return os.environ.get("DASH_TELEMETRY_DIR") or os.path.join(REPO, "sim", "telemetry")


def clean(x):
    """JSON-safe copy: NaN and infinity become null (browsers reject NaN in JSON), tuples become lists."""
    if isinstance(x, float):
        return x if math.isfinite(x) else None
    if isinstance(x, dict):
        return {str(k): clean(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [clean(v) for v in x]
    if x is None or isinstance(x, (str, int, bool)):
        return x
    return str(x)


def rel(path):
    """Path relative to the repo when inside it, else absolute."""
    p = os.path.abspath(path)
    return os.path.relpath(p, REPO) if p.startswith(REPO + os.sep) else p


# ---------------------------------------------------------------------------------------------------------------
# /api/system
# ---------------------------------------------------------------------------------------------------------------
_cpu_prev = {"total": None, "idle": None}
_gpu_cache = {"t": 0.0, "value": None, "missing_until": 0.0}
_lock = threading.Lock()


def cpu_busy():
    """Share of CPU time busy since the previous call (from /proc/stat), or None on the first call."""
    try:
        with open("/proc/stat") as f:
            parts = f.readline().split()[1:]
        vals = [int(x) for x in parts]
        idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
        total = sum(vals[:8])
    except Exception:
        return None
    with _lock:
        pt, pi = _cpu_prev["total"], _cpu_prev["idle"]
        _cpu_prev.update(total=total, idle=idle)
    if pt is None or total <= pt:
        return None
    return max(0.0, min(1.0, 1.0 - (idle - pi) / (total - pt)))


def meminfo():
    out = {}
    try:
        with open("/proc/meminfo") as f:
            for line in f:
                k, _, rest = line.partition(":")
                parts = rest.split()
                if parts:
                    out[k] = int(parts[0]) * 1024
    except Exception:
        return None
    total = out.get("MemTotal")
    avail = out.get("MemAvailable", out.get("MemFree"))
    if not total:
        return None
    return {"total": total, "available": avail, "used": total - (avail or 0),
            "swap_total": out.get("SwapTotal"), "swap_used": (out.get("SwapTotal") or 0) - (out.get("SwapFree") or 0)}


def _num(s):
    s = s.strip()
    try:
        return float(s)
    except ValueError:
        return None


def gpu():
    now = time.time()
    with _lock:
        if now < _gpu_cache["missing_until"]:
            return None
        if now - _gpu_cache["t"] < 1.5:
            return _gpu_cache["value"]
    cmd = ["nvidia-smi", "--query-gpu=name,utilization.gpu,memory.used,memory.total,temperature.gpu,power.draw,power.limit",
           "--format=csv,noheader,nounits"]
    value, missing = None, False
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=2)
        if r.returncode == 0:
            gpus = []
            for line in r.stdout.strip().splitlines():
                f = [x.strip() for x in line.split(",")]
                if len(f) < 6:
                    continue
                power, limit = _num(f[5]), _num(f[6]) if len(f) > 6 else None
                if power is not None and power > 1.5 * (limit or 400):   # nvidia-smi can misread on GPU wake (590 W seen)
                    power = None
                gpus.append({"name": f[0], "util_pct": _num(f[1]), "mem_used_mib": _num(f[2]),
                             "mem_total_mib": _num(f[3]), "temp_c": _num(f[4]), "power_w": power})
            value = gpus or None
    except FileNotFoundError:
        missing = True
    except Exception:
        value = None
    with _lock:
        _gpu_cache.update(t=now, value=value)
        if missing:
            _gpu_cache["missing_until"] = now + 60
    return value


def _uptime():
    try:
        with open("/proc/uptime") as f:
            return float(f.read().split()[0])
    except Exception:
        return None


def _proc_stat(pid):
    """(ppid, cpu seconds, age seconds, rss bytes) from /proc/<pid>/stat."""
    with open(f"/proc/{pid}/stat") as f:
        s = f.read()
    rest = s[s.rfind(")") + 2:].split()     # fields after the command name, starting at field 3 (state)
    ppid = int(rest[1])
    cpu = (int(rest[11]) + int(rest[12])) / CLK_TCK
    start = int(rest[19]) / CLK_TCK
    rss = int(rest[21]) * PAGE
    up = _uptime()
    return ppid, cpu, (up - start) if up else None, rss


def short_cmd(argv):
    exe = os.path.basename(argv[0])
    words = [exe]
    for a in argv[1:]:
        words.append(os.path.basename(a) if a.endswith(".py") else a)
    s = " ".join(words)
    return s if len(s) <= 160 else s[:157] + "..."


def processes():
    found = {}
    me = os.getpid()
    try:
        pids = [p for p in os.listdir("/proc") if p.isdigit()]
    except Exception:
        return []
    for p in pids:
        pid = int(p)
        if pid == me:
            continue
        try:
            with open(f"/proc/{p}/cmdline", "rb") as f:
                raw = f.read()
        except Exception:
            continue
        argv = [a.decode("utf-8", "replace") for a in raw.split(b"\0") if a]
        if not argv:
            continue
        exe = os.path.basename(argv[0])
        joined = " ".join(argv)
        is_python = exe.startswith("python") or (len(argv) > 1 and os.path.basename(argv[1]).startswith("python"))
        hit = is_python and (any(w in joined.lower() for w in PROC_WORDS) or KIT_RE.search(joined))
        if not hit and exe in ("kit", "isaac-sim.sh", "isaac-sim"):
            hit = True
        if not hit:
            continue
        try:
            ppid, cpu, age, rss = _proc_stat(p)
        except Exception:
            ppid, cpu, age, rss = None, None, None, None
        found[pid] = {"pid": pid, "ppid": ppid, "cmd": short_cmd(argv), "cpu_s": cpu, "age_s": age, "rss": rss}
    # fold Pool workers (same matching command, parent also listed) into their parent
    out = []
    for pid, info in sorted(found.items()):
        if info["ppid"] in found:
            continue
        kids = [k for k in found.values() if k["ppid"] == pid]
        info = dict(info, workers=len(kids),
                    workers_cpu_s=sum(k["cpu_s"] or 0 for k in kids),
                    workers_rss=sum(k["rss"] or 0 for k in kids))
        total_cpu = (info["cpu_s"] or 0) + info["workers_cpu_s"]
        info["cpu_pct"] = 100 * total_cpu / info["age_s"] if info["age_s"] else None
        out.append(info)
    return out


def api_system():
    try:
        load = list(os.getloadavg())
    except Exception:
        load = None
    try:
        du = shutil.disk_usage(REPO)
        disk = {"path": REPO, "total": du.total, "used": du.used, "free": du.free}
    except Exception:
        disk = None
    return {"time": time.time(), "hostname": HOST, "cpu_count": os.cpu_count(), "load": load,
            "cpu_busy": cpu_busy(), "memory": meminfo(), "disk": disk, "gpu": gpu(), "processes": processes()}


# ---------------------------------------------------------------------------------------------------------------
# /api/runs and /api/run/<id>
# ---------------------------------------------------------------------------------------------------------------
_run_cache = {}


def _pid_alive(pid):
    try:
        return os.path.exists(f"/proc/{int(pid)}")
    except Exception:
        return None


def read_lines(path):
    lines = []
    with open(path, encoding="utf-8", errors="replace") as f:
        for raw in f:
            raw = raw.strip()
            if not raw:
                continue
            try:
                d = json.loads(raw)
            except ValueError:
                continue
            if isinstance(d, dict):
                lines.append(d)
    return lines


def _first_last(path, size):
    """First and last JSON lines of a big file without reading all of it."""
    with open(path, "rb") as f:
        first = f.readline()
        f.seek(max(0, size - 65536))
        tail = f.read().splitlines()
    out = []
    for raw in [first] + tail[::-1]:
        try:
            d = json.loads(raw)
            if isinstance(d, dict):
                out.append(d)
        except ValueError:
            continue
        if len(out) == 2:
            break
    return out


def summarize_run(path):
    st = os.stat(path)
    key = (st.st_mtime_ns, st.st_size)
    hit = _run_cache.get(path)
    if hit and hit[0] == key:
        base = hit[1]
    else:
        if st.st_size > 20_000_000:
            lines = _first_last(path, st.st_size)
        else:
            lines = read_lines(path)
        if not lines:
            return None
        first, last = lines[0], lines[-1]
        metrics = {}
        for ln in lines:       # latest value of every metric seen (an end line may carry only a few)
            if isinstance(ln.get("metrics"), dict):
                metrics.update(ln["metrics"])
        spark = [[ln.get("t"), ln.get("done")] for ln in lines
                 if isinstance(ln.get("done"), (int, float)) and isinstance(ln.get("t"), (int, float))]
        if len(spark) > 80:
            step = len(spark) / 80
            spark = [spark[int(i * step)] for i in range(80)] + [spark[-1]]
        ended = next((ln for ln in reversed(lines) if ln.get("event") in ("end", "error")), None)
        base = {"run": first.get("run") or os.path.basename(path)[:-6], "file": rel(path),
                "kind": first.get("kind"), "label": first.get("label") or last.get("label"),
                "host": first.get("host"), "pid": first.get("pid"), "args": first.get("args"),
                "started": first.get("t"), "updated": last.get("t"), "event": last.get("event"),
                "done": last.get("done"), "total": last.get("total") if last.get("total") is not None else first.get("total"),
                "elapsed_s": last.get("elapsed_s"), "eta_s": last.get("eta_s"), "rate_per_s": last.get("rate_per_s"),
                "metrics": metrics, "msg": (ended or {}).get("msg"), "end_event": (ended or {}).get("event"),
                "n_lines": len(lines), "spark": spark}
        _run_cache[path] = (key, base)
    r = dict(base)
    now = time.time()
    upd = r.get("updated") or st.st_mtime
    r["age_s"] = max(0.0, now - upd)
    done, total = r.get("done"), r.get("total")
    r["progress"] = (done / total) if isinstance(done, (int, float)) and isinstance(total, (int, float)) and total > 0 else None
    if r["end_event"] == "end":
        status = "done"
    elif r["end_event"] == "error":
        status = "error"
    elif r["age_s"] > STALE_S:
        status = "stale"
    elif r.get("host") == HOST and r.get("pid") and _pid_alive(r["pid"]) is False:
        status = "stopped"
    else:
        status = "running"
    r["status"] = status
    eta = r.get("eta_s")
    r["eta_now_s"] = max(0.0, eta - r["age_s"]) if status == "running" and isinstance(eta, (int, float)) else None
    r["duration_s"] = (upd - r["started"]) if r.get("started") else r.get("elapsed_s")
    return r


def api_runs():
    d = telemetry_dir()
    runs = []
    if os.path.isdir(d):
        for name in os.listdir(d):
            if not name.endswith(".jsonl"):
                continue
            try:
                s = summarize_run(os.path.join(d, name))
            except Exception as e:
                s = {"run": name[:-6], "status": "unreadable", "msg": str(e)}
            if s:
                runs.append(s)
    runs.sort(key=lambda r: r.get("started") or 0, reverse=True)
    return {"dir": rel(d), "time": time.time(), "stale_after_s": STALE_S, "host": HOST, "runs": runs}


def api_run(run_id):
    if not RUN_ID_RE.match(run_id):
        return 400, {"error": "bad run id"}
    path = os.path.join(telemetry_dir(), run_id + ".jsonl")
    if not os.path.isfile(path):
        return 404, {"error": f"no run {run_id}"}
    return 200, {"run": run_id, "file": rel(path), "summary": summarize_run(path), "lines": read_lines(path)}


# ---------------------------------------------------------------------------------------------------------------
# /api/results
# ---------------------------------------------------------------------------------------------------------------
RESULT_FILES = {"robustness.json": "robustness", "speed_gate.json": "speed_gate", "all96_summary.json": "all96_summary",
                "selfmask_eval.json": "selfmask_eval",
                "records.json": "records", "approaches.json": "approaches_meta", "approaches_meta.json": "approaches_meta"}
_result_cache = {}


def _meta_from_head(path):
    """approaches.json starts with {"meta": {...}, "trials": [...]}: decode the meta object from the first bytes."""
    with open(path, encoding="utf-8") as f:
        head = f.read(65536)
    m = re.match(r'\s*\{\s*"meta"\s*:\s*', head)
    if not m:
        return None
    obj, _ = json.JSONDecoder().raw_decode(head, m.end())
    return obj if isinstance(obj, dict) else None


def parse_result(path, kind):
    if kind == "approaches_meta":
        meta = _meta_from_head(path)
        if meta is None:
            with open(path, encoding="utf-8") as f:
                meta = json.load(f).get("meta")
        return {"meta": meta}
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    if kind == "odometry_drift":
        if not isinstance(data, dict):
            raise ValueError("expected a JSON object with meta and summary")
        return {"meta": data.get("meta"), "summary": data.get("summary")}
    if kind == "speed_gate":
        keep = ("meta", "chosen", "constant", "gate", "sweep", "assumed", "tune_seeds", "test_seeds", "dirs")
        return {k: data.get(k) for k in keep if k in data}
    if kind == "records":
        return {k: data.get(k) for k in ("n_trials", "seed0", "seconds") if k in data}
    return data


def result_roots():
    roots = []
    try:
        for name in sorted(os.listdir(SANDBOX)):
            p = os.path.join(SANDBOX, name)
            if name.startswith("results") and os.path.isdir(p):
                roots.append(p)
    except Exception:
        pass
    if os.path.isdir(SUMMARIES):
        roots.append(SUMMARIES)
    if os.path.isdir(FIDUCIAL_RESULTS):
        roots.append(FIDUCIAL_RESULTS)
    if os.path.isdir(ODOMETRY_RESULTS):
        roots.append(ODOMETRY_RESULTS)
    if os.path.isdir(AUTONOMY_RUNS):
        roots.append(AUTONOMY_RUNS)
    return roots


def result_kind(dirpath, name):
    """Kind of a result file by its name; summary.json is generic, so it counts only in a folder whose name says what it is:
    isaac_tags* (fiducial tag table), drift_* (odometry drift, sim/odometry), isaac_vo* (Isaac visual odometry, format not fixed yet),
    or autonomy_runs (arena.sh runs in sim/autonomy_runs)."""
    if name == "summary.json":
        folder = os.path.basename(dirpath)
        parent_folder = os.path.basename(os.path.dirname(dirpath))
        if folder.startswith("isaac_tags"):
            return "isaac_tags"
        if folder.startswith("drift_"):
            return "odometry_drift"
        if folder.startswith("isaac_vo"):
            return "isaac_vo"
        if parent_folder == "autonomy_runs":
            return "autonomy_runs"
        return None
    return RESULT_FILES.get(name)


def api_results():
    items = []
    for root in result_roots():
        base_depth = root.rstrip(os.sep).count(os.sep)
        for dirpath, dirnames, filenames in os.walk(root):
            depth = dirpath.rstrip(os.sep).count(os.sep) - base_depth
            dirnames[:] = sorted(d for d in dirnames if not d.startswith(".") and d != "__pycache__")
            if depth >= RESULT_DEPTH:
                dirnames[:] = []
            for name in sorted(filenames):
                kind = result_kind(dirpath, name)
                if not kind:
                    continue
                path = os.path.join(dirpath, name)
                try:
                    st = os.stat(path)
                    key = (st.st_mtime_ns, st.st_size)
                    hit = _result_cache.get(path)
                    if hit and hit[0] == key:
                        content = hit[1]
                    else:
                        content = clean(parse_result(path, kind))
                        _result_cache[path] = (key, content)
                    items.append({"path": rel(path), "root": rel(root), "type": kind, "mtime": st.st_mtime,
                                  "size": st.st_size, "content": content})
                except Exception as e:
                    items.append({"path": rel(path), "root": rel(root), "type": kind, "error": str(e)})
    return {"roots": [rel(r) for r in result_roots()], "time": time.time(), "items": items}


# ---------------------------------------------------------------------------------------------------------------
# /api/params
# ---------------------------------------------------------------------------------------------------------------
_params_state = {"mtime": None, "module": None}
STATUSES = ("Verified", "Inferred", "Assumed")


def load_params():
    path = os.path.join(SANDBOX, "params.py")
    mtime = os.stat(path).st_mtime_ns
    with _lock:
        if _params_state["module"] is not None and _params_state["mtime"] == mtime:
            return _params_state["module"], path
        if SANDBOX not in sys.path:
            sys.path.insert(0, SANDBOX)
        mod = sys.modules.get("params")
        if mod is not None and os.path.abspath(getattr(mod, "__file__", "")) == os.path.abspath(path):
            mod = importlib.reload(mod)
        else:
            mod = importlib.import_module("params")
        _params_state.update(mtime=mtime, module=mod)
        return mod, path


def _status_word(s):
    s = str(s or "").strip()
    for w in STATUSES:
        if s.lower().startswith(w.lower()):
            return w
    return "Unlabelled"


def api_params():
    mod, path = load_params()
    tables = []
    counts = {s: 0 for s in STATUSES}
    for tname, table in mod.all_tables().items():
        entries = []
        for key, e in table.items():
            if all(hasattr(e, a) for a in ("value", "unit", "status", "source")):
                status = e.status if e.status in STATUSES else _status_word(e.status)
                ent = {"name": key, "value": clean(e.value), "unit": e.unit, "status": status,
                       "status_text": e.status, "source": e.source}
            elif isinstance(e, dict):   # plain-dict tables such as LIDAR_PROFILES
                note = e.get("status", "")
                ent = {"name": key, "value": clean({k: v for k, v in e.items() if k != "status"}), "unit": "",
                       "status": _status_word(note), "status_text": note, "source": note}
            else:
                ent = {"name": key, "value": clean(e), "unit": "", "status": "Unlabelled", "status_text": "",
                       "source": ""}
            counts[ent["status"]] = counts.get(ent["status"], 0) + 1
            entries.append(ent)
        tables.append({"name": tname, "entries": entries})
    return {"file": rel(path), "mtime": os.stat(path).st_mtime, "tables": tables, "counts": counts,
            "total": sum(counts.values())}


# ---------------------------------------------------------------------------------------------------------------
# /api/research
# ---------------------------------------------------------------------------------------------------------------
def api_research():
    if not os.path.isfile(RESEARCH_INDEX):
        return {"generated": None, "root": None, "files": [], "checks": [], "totals": {}, "categories": [],
                "note": "research_index.json not found; run make_research_index.py"}
    with open(RESEARCH_INDEX, encoding="utf-8") as f:
        data = json.load(f)
    data["index_file"] = rel(RESEARCH_INDEX)
    data["index_mtime"] = os.stat(RESEARCH_INDEX).st_mtime
    return data


# ---------------------------------------------------------------------------------------------------------------
# /api/planner
# ---------------------------------------------------------------------------------------------------------------
def api_planner():
    """Planner head-to-head benchmark results. Reads summary.json and nav2/*.json files fresh."""
    import numpy as np
    planner_root = os.path.join(REPO, "sim", "planner", "results")
    items = []

    if not os.path.isdir(planner_root):
        return {"root": rel(planner_root), "time": time.time(), "items": items}

    # Find all summary.json files (from compare.py) and nav2/*.json files (Nav2 live runs)
    summary_files = []
    nav2_files = []

    for dirpath, dirnames, filenames in os.walk(planner_root):
        dirnames[:] = sorted(d for d in dirnames if not d.startswith(".") and d != "__pycache__")
        for name in sorted(filenames):
            if name == "summary.json":
                summary_files.append(os.path.join(dirpath, name))
            elif name.endswith(".json") and "nav2" in dirpath:
                nav2_files.append(os.path.join(dirpath, name))

    # Process summary.json files (these have computed summaries for custom planners)
    for path in summary_files:
        try:
            st = os.stat(path)
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
        except (json.JSONDecodeError, IOError, OSError) as e:
            items.append({"path": rel(path), "error": f"Could not read: {type(e).__name__}"})
            continue

        summary = data.get("summary", [])
        meta = data.get("meta", {})

        # Each summary item is a row: {slip, planner, missions, complete, legs_with_contact, mission_time_median, mission_time_p95, ...}
        for row in summary:
            items.append({
                "path": rel(path),
                "mtime": st.st_mtime,
                "type": "summary",
                "planner": row.get("planner", "unknown"),
                "slip": row.get("slip"),
                "missions": row.get("missions"),
                "complete": row.get("complete"),
                "legs_with_contact": row.get("legs_with_contact"),
                "mission_time_median": row.get("mission_time_median"),
                "mission_time_p95": row.get("mission_time_p95"),
                "plan_s_median": row.get("plan_s_median"),
                "plan_s_p95": row.get("plan_s_p95"),
                "dump_on_target": row.get("dump_on_target"),
            })

    # Process nav2 files (live runs that grow over time)
    for path in nav2_files:
        try:
            st = os.stat(path)
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
        except (json.JSONDecodeError, IOError, OSError) as e:
            items.append({"path": rel(path), "error": f"Could not read: {type(e).__name__}"})
            continue

        runs = data.get("runs", [])
        meta = data.get("meta", {})
        planner_label = meta.get("label", "nav2_unknown")

        # Group runs by slip level and compute stats
        by_slip = {}
        for r in runs:
            slip = r.get("slip")
            if slip not in by_slip:
                by_slip[slip] = []
            by_slip[slip].append(r)

        # Compute statistics for each slip level
        for slip, slip_runs in by_slip.items():
            missions = len(slip_runs)
            complete_runs = [r for r in slip_runs if r.get("complete", False)]
            n_complete = len(complete_runs)

            # Compute legs with contact
            legs_with_contact = 0
            for r in slip_runs:
                for leg in r.get("legs", []):
                    if leg.get("contact_s", 0) > 0:
                        legs_with_contact += 1

            # Compute mission time stats for complete runs using numpy percentile
            complete_times = [r.get("time", 0) for r in complete_runs if isinstance(r.get("time"), (int, float))]
            mission_time_median = None
            mission_time_p95 = None
            if complete_times:
                mission_time_median = float(np.median(complete_times))
                mission_time_p95 = float(np.percentile(complete_times, 95))

            items.append({
                "path": rel(path),
                "mtime": st.st_mtime,
                "type": "nav2",
                "planner": planner_label,
                "slip": slip,
                "missions": missions,
                "complete": n_complete,
                "legs_with_contact": legs_with_contact,
                "mission_time_median": mission_time_median,
                "mission_time_p95": mission_time_p95,
                "plan_s_median": None,
                "plan_s_p95": None,
                "dump_on_target": None,
                "runs": [{"seed": r.get("seed"), "slip": r.get("slip"), "complete": r.get("complete"),
                          "time": r.get("time"), "legs": [{"k": l.get("k"), "status": l.get("status"),
                                                           "time": l.get("time"), "e_xy": l.get("e_xy"),
                                                           "e_yaw": l.get("e_yaw"), "cmd_hz": l.get("cmd_hz")}
                                                         for l in r.get("legs", [])]}
                         for r in slip_runs]
            })

    return {"root": rel(planner_root), "time": time.time(), "items": items}


# ---------------------------------------------------------------------------------------------------------------
# HTTP
# ---------------------------------------------------------------------------------------------------------------
ROUTES = {"/api/system": api_system, "/api/runs": api_runs, "/api/results": api_results,
          "/api/params": api_params, "/api/research": api_research, "/api/planner": api_planner}


class Handler(BaseHTTPRequestHandler):
    server_version = "SandboxDashboard/1.0"
    verbose = False

    def log_message(self, fmt, *args):
        if self.verbose:
            sys.stderr.write("%s - %s\n" % (self.address_string(), fmt % args))

    def _send(self, code, body, ctype):
        try:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            if self.command != "HEAD":
                self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _json(self, code, obj):
        body = json.dumps(clean(obj), separators=(",", ":"), allow_nan=False).encode("utf-8")
        self._send(code, body, "application/json; charset=utf-8")

    def do_HEAD(self):
        self.do_GET()

    def do_GET(self):
        path = urlparse(self.path).path
        try:
            if path in STATIC:
                name, ctype = STATIC[path]
                with open(os.path.join(DASH, name), "rb") as f:
                    self._send(200, f.read(), ctype)
            elif path in ROUTES:
                self._json(200, ROUTES[path]())
            elif path.startswith("/api/run/"):
                code, obj = api_run(unquote(path[len("/api/run/"):]))
                self._json(code, obj)
            elif path == "/favicon.ico":
                self._send(204, b"", "image/x-icon")
            else:
                self._json(404, {"error": f"not found: {path}"})
        except Exception as e:
            if self.verbose:
                traceback.print_exc()
            try:
                self._json(500, {"error": f"{type(e).__name__}: {e}", "path": path})
            except Exception:
                pass


def main():
    ap = argparse.ArgumentParser(description="Local dashboard for the BP-1 sandbox (binds to 127.0.0.1)")
    ap.add_argument("--port", type=int, default=8765, help="port to listen on (default 8765)")
    ap.add_argument("--verbose", action="store_true", help="log every request and print tracebacks")
    a = ap.parse_args()
    Handler.verbose = a.verbose
    try:
        httpd = ThreadingHTTPServer(("127.0.0.1", a.port), Handler)
    except OSError as e:
        print(f"cannot listen on 127.0.0.1:{a.port} ({e}). Try another port: --port {a.port + 1}", file=sys.stderr)
        sys.exit(1)
    httpd.daemon_threads = True
    print(f"Dashboard at http://127.0.0.1:{a.port}/  (repo {REPO}; telemetry {telemetry_dir()}). Ctrl+C stops it.",
          flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()


if __name__ == "__main__":
    main()
