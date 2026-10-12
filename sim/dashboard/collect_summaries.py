#!/usr/bin/env python3
"""Copy small result summaries into the repo so the dashboard shows them after a git pull.

    python3 sim/dashboard/collect_summaries.py [--src /mnt/project-files/step2/bp1_sandbox] [--dry-run]

Copies robustness.json/.md, speed_gate.json/.md, selfmask_eval.json/.md, localize.json/.md, all96_summary.json, tables.md and *.png under 400 KB from the
results root into <repo>/sim/bp1_sandbox/summaries/<same relative path>/. For each approaches.json (several MB)
it writes only its "meta" object, as approaches_meta.json ({"meta": {...}}). Anything over 1 MB is skipped.
The source folder is only read. Files whose size and modification time already match are left alone.
Standard library only.
"""
import argparse
import json
import os
import re
import shutil
import sys

DASH = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(DASH))
DEFAULT_SRC = "/mnt/project-files/step2/bp1_sandbox"
DEFAULT_DEST = os.path.join(REPO, "sim", "bp1_sandbox", "summaries")
NAMES = {"robustness.json", "robustness.md", "speed_gate.json", "speed_gate.md", "all96_summary.json", "tables.md",
         "selfmask_eval.json", "selfmask_eval.md", "localize.json", "localize.md"}
PNG_MAX = 400 * 1024
FILE_MAX = 1024 * 1024
SKIP_DIRS = {"__pycache__", "usd", "tests", ".git"}


def approaches_meta(path):
    """The "meta" object of an approaches.json, read from the head of the file when it comes first."""
    with open(path, encoding="utf-8") as f:
        head = f.read(65536)
    m = re.match(r'\s*\{\s*"meta"\s*:\s*', head)
    if m:
        try:
            obj, _ = json.JSONDecoder().raw_decode(head, m.end())
            if isinstance(obj, dict):
                return obj
        except ValueError:
            pass
    with open(path, encoding="utf-8") as f:
        return json.load(f).get("meta")


def same(src, dst):
    try:
        a, b = os.stat(src), os.stat(dst)
    except OSError:
        return False
    return a.st_size == b.st_size and int(a.st_mtime) == int(b.st_mtime)


def main():
    ap = argparse.ArgumentParser(description="Copy small result summaries into sim/bp1_sandbox/summaries/")
    ap.add_argument("--src", default=DEFAULT_SRC, help=f"results root to read (default {DEFAULT_SRC})")
    ap.add_argument("--dest", default=DEFAULT_DEST, help="where the copies go (default sim/bp1_sandbox/summaries)")
    ap.add_argument("--dry-run", action="store_true", help="list what would be copied, write nothing")
    a = ap.parse_args()
    src, dest = os.path.abspath(a.src), os.path.abspath(a.dest)
    if not os.path.isdir(src):
        sys.exit(f"no results root at {src}")
    if dest == src or dest.startswith(src + os.sep):
        sys.exit("--dest must be outside --src (the source is read-only)")
    copied = skipped = unchanged = 0
    for dirpath, dirnames, filenames in os.walk(src):
        dirnames[:] = sorted(d for d in dirnames if not d.startswith(".") and d not in SKIP_DIRS)
        reldir = os.path.relpath(dirpath, src)
        for name in sorted(filenames):
            path = os.path.join(dirpath, name)
            out_dir = os.path.normpath(os.path.join(dest, reldir))
            try:
                size = os.path.getsize(path)
                if name == "approaches.json":
                    out = os.path.join(out_dir, "approaches_meta.json")
                    meta = approaches_meta(path)
                    text = json.dumps({"meta": meta, "source_size": size}, indent=1) + "\n"
                    try:
                        with open(out, encoding="utf-8") as f:
                            if f.read() == text:
                                unchanged += 1
                                continue
                    except OSError:
                        pass
                    print(f"meta  {os.path.relpath(out, dest)}  (from {size / 1e6:.1f} MB)")
                    if not a.dry_run:
                        os.makedirs(out_dir, exist_ok=True)
                        with open(out, "w", encoding="utf-8") as f:
                            f.write(text)
                        st = os.stat(path)
                        os.utime(out, (st.st_atime, st.st_mtime))
                    copied += 1
                    continue
                wanted = name in NAMES or (name.lower().endswith(".png") and size < PNG_MAX)
                if not wanted:
                    continue
                if size > FILE_MAX:
                    print(f"skip  {os.path.join(reldir, name)}  ({size / 1e6:.1f} MB, over 1 MB)")
                    skipped += 1
                    continue
                out = os.path.join(out_dir, name)
                if same(path, out):
                    unchanged += 1
                    continue
                print(f"copy  {os.path.relpath(out, dest)}  ({size / 1e3:.0f} KB)")
                if not a.dry_run:
                    os.makedirs(out_dir, exist_ok=True)
                    shutil.copy2(path, out)
                copied += 1
            except (OSError, ValueError) as e:
                print(f"error {os.path.join(reldir, name)}: {e}", file=sys.stderr)
                skipped += 1
    verb = "would write" if a.dry_run else "wrote"
    print(f"{verb} {copied} files, {unchanged} unchanged, {skipped} skipped -> {dest}")


if __name__ == "__main__":
    main()
