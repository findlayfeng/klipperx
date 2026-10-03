#!/usr/bin/env python3
"""Audit `foo.py:NNN[-MMM]` references in this repo against the pinned upstream Klipper tree.

Every module in `src/core/klippy/` (and the manuals under `docs/`) cites the upstream
Python source it was ported from, e.g. ``(`mcu.py:718-719`)``.  Those line numbers
drift: the reference may have been written against a different checkout, and the
pinned tree at `third_party/klipper/` moves on.  This script flags the ones that no
longer line up so they can be re-anchored.

How it works
------------
For each reference it looks for an anchor on the same line and in the preceding
`--window` lines (default 9, i.e. the surrounding doc-comment block):

* ``symbol``  — the comment names a class/function (`def foo` / `class Foo`) that the
  upstream file really defines, and the cited range does not match that definition's
  exact `lineno..end_lineno` (parsed with `ast`, so it is exact).  The suggestion is
  the correct range.
* ``literal`` — a backticked snippet on the same line (e.g. `` `MIN_SCHEDULE_TIME = 0.100` ``)
  is found verbatim in the upstream file, but only outside the cited range.  The
  suggestion is the line where it occurs.
* ``file``    — the cited range runs past the end of the upstream file.  This one is
  unambiguous.

Output is a candidate list, **not** a verdict: see "false positives" below.  Exit
status is 0 regardless of findings; this is a discovery tool, not a gate.

False positives (read before "fixing" anything)
-----------------------------------------------
The window deliberately over-approximates, so a large share of hits are wrong:

* an English word or Rust identifier in the window colliding with an unrelated
  upstream `def` (`move`, `reset`, `flush`, `get_status`, `__init__`, …);
* `__init__` / `get_status` appearing in several classes — only one range is kept;
* a citation that legitimately points *inside* a function (a sub-block) or at a
  module-level constant, module body, or blank-line boundary;
* a citation that deliberately points at the *body line* that implements the
  behaviour rather than at the `def` line (e.g. the `is_fileoutput()` short-circuit
  table in `regression-tests.md` cites body lines).

So: open the pinned file, read the cited range, and decide.  In this repo's first
sweep (2026-10-03, 105 files) roughly half of the window-mode hits turned out to be
candidates worth fixing and the rest were artifacts of the heuristics above.

Usage
-----
    python3 scripts/pyref-audit.py                     # whole src tree, summary
    python3 scripts/pyref-audit.py extras/display/     # substring filter
    python3 scripts/pyref-audit.py extras/display/ --detail
    python3 scripts/pyref-audit.py --docs              # manuals instead of source
    python3 scripts/pyref-audit.py --window 1          # same-line anchors only (strict)

The upstream tree must be checked out: `third_party/klipper/` (a pinned submodule).
A git worktree of this repo has that directory empty — pass
`--upstream /path/to/main/checkout/third_party/klipper` in that case.
"""

from __future__ import annotations

import argparse
import ast
import os
import re
import subprocess
import sys
from collections import defaultdict

REF = re.compile(r"([A-Za-z_][A-Za-z0-9_]*\.py):(\d+)(?:-(\d+))?")

# Docs scanned by --docs (source files under --src are scanned otherwise).
DOC_EXTRA_FILES = ("TODO.md", "README.md", "TESTING.md")


def repo_root() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    try:
        out = subprocess.run(
            ["git", "-C", here, "rev-parse", "--show-toplevel"],
            capture_output=True, text=True, check=True,
        )
        return out.stdout.strip()
    except Exception:
        return os.path.dirname(here)


_LINES: dict[str, list[str]] = {}


def read_lines(path: str) -> list[str]:
    """Line cache shared by the source files and the upstream tree."""
    if path not in _LINES:
        with open(path, encoding="utf-8", errors="replace") as fh:
            _LINES[path] = fh.readlines()
    return _LINES[path]


class Upstream:
    """Index of the pinned upstream checkout: file lookup plus symbol tables."""

    def __init__(self, root: str) -> None:
        self.root = root
        if not os.path.isdir(root):
            sys.exit(
                f"upstream tree not found: {root}\n"
                "Check out third_party/klipper, or pass --upstream <path> "
                "(a worktree of this repo has it empty)."
            )
        self.by_name: dict[str, list[str]] = defaultdict(list)
        for dirpath, _, files in os.walk(root):
            for name in files:
                if name.endswith(".py"):
                    self.by_name[name].append(os.path.join(dirpath, name))
        if not self.by_name:
            sys.exit(
                f"no Python files under {root}\n"
                "The pinned submodule is probably not checked out (a git worktree of "
                "this repo leaves it empty). Pass --upstream <main checkout>/third_party/klipper."
            )
        self._symbols: dict[str, tuple[dict, dict]] = {}

    def lines(self, path: str) -> list[str]:
        return read_lines(path)

    def resolve(self, name: str, rel_source: str) -> str | None:
        """Map `foo.py` to a path under the upstream root."""
        cands = self.by_name.get(name)
        if not cands:
            return None
        if len(cands) == 1:
            return cands[0]
        # Prefer a candidate whose directory components appear in the citing path.
        for path in cands:
            parts = os.path.relpath(path, self.root).replace(".py", "").split(os.sep)
            if any(part in rel_source for part in parts[:-1]):
                return path
        return cands[0]

    def symbols(self, path: str) -> tuple[dict[str, tuple[int, int]], dict[str, int]]:
        """(defs, assignments): class/function exact ranges, then name/attr lines."""
        if path in self._symbols:
            return self._symbols[path]
        defs: dict[str, tuple[int, int]] = {}
        assigns: dict[str, int] = {}
        try:
            tree = ast.parse("".join(self.lines(path)))
        except SyntaxError:
            self._symbols[path] = (defs, assigns)
            return defs, assigns

        def walk(node: ast.AST) -> None:
            for child in ast.iter_child_nodes(node):
                if isinstance(child, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
                    defs.setdefault(child.name, (child.lineno, child.end_lineno))
                elif isinstance(child, ast.Assign):
                    for target in child.targets:
                        if isinstance(target, ast.Name):
                            assigns.setdefault(target.id, child.lineno)
                        elif isinstance(target, ast.Attribute):
                            assigns.setdefault(target.attr, child.lineno)
                walk(child)

        walk(tree)
        self._symbols[path] = (defs, assigns)
        return defs, assigns


def overlaps(a: int, b: int, c: int, d: int) -> bool:
    """True when two inclusive ranges share at least as much as the shorter one."""
    shared = max(0, min(b, d) - max(a, c) + 1)
    return shared >= min(b - a + 1, d - c + 1)


def audit(up: Upstream, path: str, start: int, end: int, ctx: str) -> tuple[str, str, object]:
    lines = up.lines(path)
    if start < 1 or end > len(lines) or start > end:
        return "file", f"range {start}-{end} past end of file ({len(lines)} lines)", None

    defs, _assigns = up.symbols(path)
    tokens = set(re.findall(r"[A-Za-z_][A-Za-z0-9_]{2,}", ctx))

    for token in tokens:
        if token in defs:
            dstart, dend = defs[token]
            if not overlaps(start, end, dstart, dend):
                return "symbol", f"{token} spans {dstart}-{dend}", (dstart, dend)

    for literal in (m.group(1).strip() for m in re.finditer(r"`([^`]+)`", ctx)):
        if not 4 <= len(literal) <= 90 or not re.search(r"[ .=(]", literal):
            continue
        hits = [i for i, line in enumerate(lines, 1) if literal in line]
        if hits and not any(start <= h <= end for h in hits):
            return "literal", f"`{literal}` appears only at line {hits[0]}", hits[0]

    return "", "", None


def collect(args) -> tuple[int, list[tuple]]:
    root = args.root
    sources: list[str] = []
    if args.docs:
        for dirpath, _, files in os.walk(os.path.join(root, "docs")):
            sources += [os.path.join(dirpath, f) for f in files if f.endswith(".md")]
        sources += [
            os.path.join(root, name)
            for name in DOC_EXTRA_FILES
            if os.path.isfile(os.path.join(root, name))
        ]
    else:
        for dirpath, _, files in os.walk(args.src):
            sources += [os.path.join(dirpath, f) for f in files if f.endswith(".rs")]

    up = Upstream(args.upstream)
    findings: list[tuple] = []
    total = 0
    for source in sorted(sources):
        rel = os.path.relpath(source, root)
        if args.filter and args.filter not in rel:
            continue
        lines = read_lines(source)
        for lineno, line in enumerate(lines, 1):
            ctx = "".join(lines[max(0, lineno - args.window) : lineno])
            for match in REF.finditer(line):
                total += 1
                name = match.group(1)
                start = int(match.group(2))
                end = int(match.group(3)) if match.group(3) else start
                path = up.resolve(name, rel)
                if path is None:
                    findings.append(("missing", rel, lineno, match.group(0),
                                     "not found in the upstream tree", None))
                    continue
                tier, why, suggestion = audit(up, path, start, end, ctx)
                if tier:
                    findings.append((tier, rel, lineno, match.group(0), why, suggestion))
    return total, findings


def main() -> int:
    root = repo_root()
    parser = argparse.ArgumentParser(
        description="Audit upstream-Python line references against the pinned Klipper tree.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Read the module docstring for the heuristics and their false positives.",
    )
    parser.add_argument("filter", nargs="?", default="",
                        help="only scan paths containing this substring")
    parser.add_argument("--docs", action="store_true",
                        help="scan docs/**/*.md (+ TODO.md/README.md/TESTING.md) instead of the source")
    parser.add_argument("--detail", action="store_true", help="print every finding")
    parser.add_argument("--window", type=int, default=9, metavar="N",
                        help="anchor context window in lines (default 9; 1 = same line only)")
    parser.add_argument("--src", default=os.path.join(root, "src/core/klippy"),
                        help="directory scanned for .rs files")
    parser.add_argument("--upstream", default=os.path.join(root, "third_party/klipper"),
                        help="pinned upstream checkout (default: third_party/klipper)")
    parser.add_argument("--root", default=root, help=argparse.SUPPRESS)
    args = parser.parse_args()

    total, findings = collect(args)
    by_tier = defaultdict(int)
    by_file: dict[str, list[int]] = defaultdict(lambda: [0, 0])
    for tier, rel, *_ in findings:
        by_tier[tier] += 1
        by_file[rel][0 if tier == "file" else 1] += 1

    print(f"scanned references: {total}   candidates: {len(findings)}")
    for tier, label in (
        ("file", "range past end of upstream file (unambiguous)"),
        ("missing", "upstream file not found (check the name/spelling)"),
        ("symbol", "named class/def with mismatching range"),
        ("literal", "backticked snippet located elsewhere"),
    ):
        if by_tier[tier]:
            print(f"  {by_tier[tier]:4d}  {tier:8s} {label}")

    if args.detail:
        print()
        for tier, rel, lineno, ref, why, suggestion in findings:
            hint = ""
            if isinstance(suggestion, tuple):
                hint = f"  => {suggestion[0]}-{suggestion[1]}"
            elif suggestion:
                hint = f"  => {suggestion}"
            print(f"{tier:8s} {rel}:{lineno}: {ref}  ({why}){hint}")
    elif by_file:
        print("\nper file ('!' = unambiguous 'file' tier):")
        for rel, (hard, soft) in sorted(by_file.items(), key=lambda kv: -(kv[1][0] * 1000 + kv[1][1])):
            print(f"  {hard:3d}! {soft:4d}  {rel}")

    print("\nNOTE: candidates only — verify each against the pinned file before editing;")
    print("      window anchors are deliberately noisy (see the module docstring).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
