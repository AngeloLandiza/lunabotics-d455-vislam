#!/usr/bin/env python3
"""Build sim/dashboard/research_index.json from the research folder (read-only).

    python3 sim/dashboard/make_research_index.py [--root /mnt/project-files/research] [--out research_index.json]

Research files: <step>/*.md (not verify/). For each: step, file name, first heading as title, size in bytes, and the
first line when it is a "> Check status" note (plus the check file that note names).
Check files: <step>/verify/verify-*.md. Their "## Result counts" table gives rows per research file; columns are
matched by name, case-insensitive, so slightly different headers still parse. A check file with no table but lines
such as "File 19 (90 claim rows): Confirmed 71; Mismatch 5; ..." is parsed from that text instead.
Totals are summed per step and overall. Standard library only; the research folder is never written.
"""
import argparse
import json
import os
import re
import sys
import time

DEFAULT_ROOT = "/mnt/project-files/research"
DEFAULT_OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "research_index.json")
CATEGORIES = ["confirmed", "partly_confirmed", "mismatch", "not_found", "could_not_open", "not_checked"]
LABELS = {"confirmed": "Confirmed", "partly_confirmed": "Partly confirmed", "mismatch": "Mismatch",
          "not_found": "Not found", "could_not_open": "Could not open", "not_checked": "Not checked"}


def column_key(header):
    """Map a table header cell to a field name, or None."""
    h = re.sub(r"\s+", " ", header.strip().lower())
    if h.startswith("file"):
        return "file"
    if h.startswith("partly confirmed") or h.startswith("partially confirmed"):
        return "partly_confirmed"
    if h.startswith("confirmed"):
        return "confirmed"
    if h.startswith("mismatch"):
        return "mismatch"
    if h.startswith("not found"):
        return "not_found"
    if h.startswith("could not open"):
        return "could_not_open"
    if h.startswith("not checked"):
        return "not_checked"
    if h in ("rows", "rows checked", "claims", "claims checked", "claim rows", "total", "total rows", "rows total"):
        return "rows"
    return None


def leading_int(cell):
    m = re.match(r"\s*\**\s*(\d+)", cell)
    return int(m.group(1)) if m else None


def first_heading(text):
    for line in text.splitlines():
        if line.startswith("# "):
            return line[2:].strip()
    return None


def section(text, heading_prefix):
    """Lines of the '## ...' section whose heading starts with heading_prefix (case-insensitive)."""
    out, inside = [], False
    for line in text.splitlines():
        if line.startswith("## "):
            if inside:
                break
            inside = line[3:].strip().lower().startswith(heading_prefix.lower())
            continue
        if inside:
            out.append(line)
    return out if inside or out else None


def split_row(line):
    s = line.strip()
    if s.startswith("|"):
        s = s[1:]
    if s.endswith("|"):
        s = s[:-1]
    return [c.strip() for c in s.split("|")]


def parse_table(lines):
    """First markdown table in lines -> (columns found, list of row dicts with raw cells)."""
    rows, header, keys = [], None, None
    for line in lines:
        if not line.strip().startswith("|"):
            if header is not None and rows:
                break
            continue
        cells = split_row(line)
        if header is None:
            header, keys = cells, [column_key(c) for c in cells]
            continue
        if all(re.fullmatch(r":?-{2,}:?", c.replace(" ", "")) for c in cells if c):
            continue
        rows.append({k: c for k, c in zip(keys, cells) if k})
    return header, keys, rows


TEXT_ROW = re.compile(r"^\s*(?:[-*]\s*)?File\s+([0-9A-Za-z._-]+)\s*(?:\(([^)]*)\))?\s*:\s*(.+)$", re.I)


def parse_text(lines):
    """Rows written as 'File 19 (90 claim rows): Confirmed 71; Mismatch 5; ...'."""
    rows = []
    for line in lines:
        m = TEXT_ROW.match(line)
        if not m:
            continue
        row = {"file": m.group(1)}
        if m.group(2):
            n = re.search(r"(\d+)\s+(?:claim\s+)?rows?", m.group(2), re.I)
            if n:
                row["rows"] = n.group(1)
        for part in re.split(r"[;,]", m.group(3)):
            pm = re.match(r"\s*([A-Za-z][A-Za-z ]*?)\s*:?\s*(\d+)", part)
            if pm:
                k = column_key(pm.group(1))
                if k and k != "file":
                    row[k] = pm.group(2)
        if any(k in row for k in CATEGORIES):
            rows.append(row)
    return rows


def research_files(root, step):
    d = os.path.join(root, step)
    try:
        return sorted(n for n in os.listdir(d) if n.endswith(".md") and os.path.isfile(os.path.join(d, n)))
    except OSError:
        return []


def resolve(cell, step, files):
    """'01 lidar-occlusion', '03-fusion.md' or '19' -> 'step1/01-...md' when a research file has that number."""
    m = re.match(r"\s*\**\s*(\d{1,3})", cell)
    if not m:
        return None
    num = m.group(1).zfill(2)
    for n in files:
        if n.startswith(num + "-") or n.startswith(num + "_"):
            return f"{step}/{n}"
    return None


def empty_totals():
    t = {"rows": 0, "files": 0}
    t.update({c: 0 for c in CATEGORIES})
    return t


def add(total, row):
    total["files"] += 1
    total["rows"] += row["rows"] or 0
    for c in CATEGORIES:
        total[c] += row.get(c) or 0


def parse_check(path, rel, step, files):
    with open(path, encoding="utf-8", errors="replace") as f:
        text = f.read()
    entry = {"step": step, "file": os.path.basename(path), "path": rel, "title": first_heading(text),
             "size": os.path.getsize(path), "parse": "none", "columns": [], "rows": [], "warnings": [],
             "reported_total": None}
    lines = section(text, "Result counts")
    if lines is None:
        entry["warnings"].append("no '## Result counts' section")
        return entry
    header, keys, raw = parse_table(lines)
    if raw:
        entry["parse"] = "table"
        entry["columns"] = [h for h, k in zip(header, keys) if k]
        unknown = [h for h, k in zip(header, keys) if not k]
        if unknown:
            entry["warnings"].append("columns not used: " + ", ".join(unknown))
    else:
        raw = parse_text(lines)
        if raw:
            entry["parse"] = "text"
        else:
            entry["warnings"].append("no table or 'File N: Confirmed ...' lines in Result counts")
            return entry
    for r in raw:
        name = r.get("file", "").strip().strip("*").strip()
        vals = {c: leading_int(r[c]) if c in r else None for c in CATEGORIES}
        given = leading_int(r["rows"]) if "rows" in r else None
        summed = sum(v for v in vals.values() if v is not None)
        if name.lower().startswith("total"):
            entry["reported_total"] = {"rows": given if given is not None else summed, **vals}
            continue
        target = resolve(name, step, files)
        if target and re.fullmatch(r"\d+", name):     # 'File 19' in prose: show the real file name
            name = os.path.basename(target)
        row = {"file": name, "research_file": target, "rows": given if given is not None else summed,
               "rows_given": given is not None, **vals}
        if given is not None and given != summed:
            entry["warnings"].append(f"{name}: rows column says {given}, categories add up to {summed}")
        entry["rows"].append(row)
    entry["totals"] = empty_totals()
    for row in entry["rows"]:
        add(entry["totals"], row)
    return entry


def main():
    ap = argparse.ArgumentParser(description="Index research files and their check results for the dashboard")
    ap.add_argument("--root", default=DEFAULT_ROOT, help=f"research folder (default {DEFAULT_ROOT})")
    ap.add_argument("--out", default=DEFAULT_OUT, help="output JSON (default sim/dashboard/research_index.json)")
    a = ap.parse_args()
    root = os.path.abspath(a.root)
    if not os.path.isdir(root):
        sys.exit(f"no research folder at {root}")
    steps = sorted(n for n in os.listdir(root) if os.path.isdir(os.path.join(root, n)) and not n.startswith("."))
    files, checks = [], []
    totals = {"overall": empty_totals()}
    for step in steps:
        names = research_files(root, step)
        for n in names:
            p = os.path.join(root, step, n)
            try:
                with open(p, encoding="utf-8", errors="replace") as f:
                    text = f.read()
            except OSError as e:
                files.append({"step": step, "file": n, "path": f"{step}/{n}", "error": str(e)})
                continue
            first = text.splitlines()[0] if text else ""
            note = first.lstrip("> ").strip() if first.lstrip().startswith("> Check status") else None
            ref = re.search(r"(verify/verify-[\w.-]+?\.md)", note or "")
            files.append({"step": step, "file": n, "path": f"{step}/{n}", "title": first_heading(text),
                          "size": os.path.getsize(p), "check_note": note,
                          "check_file": f"{step}/{ref.group(1)}" if ref else None})
        vdir = os.path.join(root, step, "verify")
        if os.path.isdir(vdir):
            totals[step] = empty_totals()
            for n in sorted(os.listdir(vdir)):
                if not (n.startswith("verify-") and n.endswith(".md")):
                    continue
                try:
                    c = parse_check(os.path.join(vdir, n), f"{step}/verify/{n}", step, names)
                except OSError as e:
                    c = {"step": step, "file": n, "path": f"{step}/verify/{n}", "parse": "none", "rows": [],
                         "warnings": [str(e)]}
                checks.append(c)
                for row in c["rows"]:
                    add(totals[step], row)
                    add(totals["overall"], row)
    out = {"generated": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "root": root, "categories": CATEGORIES,
           "labels": LABELS, "files": files, "checks": checks, "totals": totals}
    tmp = a.out + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=1)
        f.write("\n")
    os.replace(tmp, a.out)
    n_rows = sum(len(c["rows"]) for c in checks)
    print(f"{len(files)} research files, {len(checks)} check files ({n_rows} per-file rows) -> {a.out}")
    for c in checks:
        for w in c.get("warnings", []):
            print(f"  note: {c['path']}: {w}")


if __name__ == "__main__":
    main()
