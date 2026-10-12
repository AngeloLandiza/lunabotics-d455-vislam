"""Run progress for the dashboard (sim/dashboard/server.py): one JSON line per event in <dir>/<run_id>.jsonl.

    from telemetry import Telemetry
    tel = Telemetry(kind="approach", total=48, args=vars(a), label="floor waves 4 cm")
    tel.progress(done, arenas_per_min=2.1)   # call it in the parent as Pool results arrive
    tel.end(seconds=1028.3)                  # or tel.error("what went wrong")

The directory is $DASH_TELEMETRY_DIR if set, else <repo>/sim/telemetry/ (created if missing).
Line fields: t (epoch s), run, kind, label, event (start|progress|end|error), done, total, elapsed_s, eta_s,
rate_per_s, host, pid, args (start line only), metrics (numbers or short strings), msg.

Telemetry never stops a run: every file operation is guarded, and when the directory cannot be written it warns
once on stderr and does nothing after that. Only the process that created the object writes, so a copy inherited
by a forked Pool worker stays silent. Standard library only.
"""
import atexit
import json
import math
import os
import socket
import sys
import time
from datetime import datetime, timezone

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_DIR = os.path.join(_REPO, "sim", "telemetry")
_STR_MAX = 200


def telemetry_dir():
    return os.environ.get("DASH_TELEMETRY_DIR") or DEFAULT_DIR


def _scalar(x):
    """A JSON-safe number or short string."""
    try:
        if hasattr(x, "item") and not isinstance(x, (str, bytes)):
            x = x.item()  # numpy scalar
    except Exception:
        pass
    if isinstance(x, bool):
        return int(x)
    if isinstance(x, int):
        return x
    if isinstance(x, float):
        return x if math.isfinite(x) else None
    if x is None:
        return None
    s = x if isinstance(x, str) else str(x)
    return s if len(s) <= _STR_MAX else s[: _STR_MAX - 3] + "..."


def _metrics(m):
    out = {}
    for k, val in (m or {}).items():
        try:
            out[str(k)[:64]] = _scalar(val)
        except Exception:
            pass
    return out


def _args(a):
    if a is None:
        return None
    try:
        if not isinstance(a, dict):
            a = vars(a)  # argparse.Namespace
    except TypeError:
        return {"value": _scalar(a)}
    out = {}
    for k, val in a.items():
        try:
            if isinstance(val, (list, tuple)) and len(val) <= 20:
                out[str(k)] = [_scalar(x) for x in val]
            else:
                out[str(k)] = _scalar(val)
        except Exception:
            pass
    return out


class Telemetry:
    def __init__(self, kind, total=None, args=None, label="", run_id=None):
        self.kind = str(kind)
        self.total = total
        self.label = str(label or "")
        self.pid = os.getpid()
        self.t0 = time.time()
        self.done = 0
        self.finished = False
        self._disabled = False
        self._warned = False
        try:
            self.host = socket.gethostname()
        except Exception:
            self.host = "unknown"
        stamp = datetime.fromtimestamp(self.t0, timezone.utc).strftime("%Y%m%d-%H%M%S")
        self.run_id = run_id or f"{self.kind}-{stamp}-{self.pid}"
        self.path = None
        try:
            d = telemetry_dir()
            os.makedirs(d, exist_ok=True)
            self.path = os.path.join(d, self.run_id + ".jsonl")
        except Exception as e:
            self._disable(e)
        self._write("start", args=_args(args))
        try:
            atexit.register(self._at_exit)
        except Exception:
            pass

    # public API -------------------------------------------------------------------------------------------------
    def progress(self, done, **metrics):
        """Record that `done` of `total` items are finished; extra keywords are metrics shown on the dashboard."""
        try:
            self.done = done
        except Exception:
            pass
        self._write("progress", metrics=metrics)

    def end(self, **metrics):
        """The run finished normally. A `done=` keyword overrides the count (defaults to total when it is known)."""
        if "done" in metrics:
            self.done = metrics.pop("done")
        elif self.total is not None:
            self.done = self.total
        self._write("end", metrics=metrics)
        self.finished = True

    def error(self, msg, **metrics):
        """The run failed or stopped early."""
        self._write("error", metrics=metrics, msg=msg)
        self.finished = True

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        if not self.finished:
            if exc_type is None:
                self.end()
            else:
                self.error(f"{exc_type.__name__}: {exc}")
        return False

    # internals --------------------------------------------------------------------------------------------------
    def _at_exit(self):
        if not self.finished and os.getpid() == self.pid:
            self.error("process exited without calling end()")

    def _disable(self, e):
        self._disabled = True
        if not self._warned:
            self._warned = True
            try:
                print(f"telemetry: cannot write to {telemetry_dir()} ({e}); continuing without telemetry",
                      file=sys.stderr, flush=True)
            except Exception:
                pass

    def _write(self, event, metrics=None, msg=None, args=None):
        if self._disabled or self.path is None:
            return
        try:
            if os.getpid() != self.pid:      # a forked worker holding a copy: only the creator writes
                return
            now = time.time()
            elapsed = now - self.t0
            done, total = _scalar(self.done), _scalar(self.total)
            rate = eta = None
            if isinstance(done, (int, float)) and done > 0 and elapsed > 0:
                rate = done / elapsed
                if isinstance(total, (int, float)) and event == "progress":
                    eta = max(0.0, (total - done) / rate)
            if event == "end":
                eta = 0.0
            line = {"t": round(now, 3), "run": self.run_id, "kind": self.kind, "label": self.label, "event": event,
                    "done": done, "total": total, "elapsed_s": round(elapsed, 3),
                    "eta_s": None if eta is None else round(eta, 1),
                    "rate_per_s": None if rate is None else round(rate, 6),
                    "host": self.host, "pid": self.pid}
            if event == "start":
                line["args"] = args
            line["metrics"] = _metrics(metrics)
            line["msg"] = None if msg is None else _scalar(msg)
            text = json.dumps(line, default=str, allow_nan=False) + "\n"
        except Exception as e:
            self._disable(e)
            return
        try:
            with open(self.path, "a", encoding="utf-8") as f:
                f.write(text)
        except Exception as e:
            self._disable(e)
