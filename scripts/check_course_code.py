#!/usr/bin/env python3
"""Fact-check the Python and MicroPython code in course lessons.

Every lesson code block is checked in three passes:

1. **Parse.** Each block must be valid Python 3 syntax.
2. **API check (static).** MicroPython blocks are checked against what the
   Raspberry Pi Pico (rp2) firmware actually has: no ``tm_hour`` on
   ``time.localtime()`` (it returns a plain tuple), no ``time.strftime``, no
   ``machine.ADC.read()``, no CPython-only modules such as ``datetime``.
3. **Run.** If a MicroPython unix binary is available (``--micropython`` or
   ``$MICROPYTHON``), each MicroPython block is run on it with strict
   stand-ins for ``machine``, ``rp2``, ``network`` and friends. Calling a
   method the Pico does not have fails exactly as it would on the board.
   ``while True`` loops stop after a few ``sleep`` calls.

Usage::

    python3 scripts/check_course_code.py source/pico_temp_sensor
    python3 scripts/check_course_code.py source/*/07_*.md --micropython ~/micropython
    python3 scripts/check_course_code.py --changed origin/main

Build the MicroPython binary once with ``scripts/get_micropython.sh``; the
checker finds it in ~/.cache/kevsrobots/micropython automatically.

Markers in the lesson, on the line(s) just above a code fence::

    <!-- code-check: skip -->         don't check this block at all
    <!-- code-check: fragment -->     parse and API check only, don't run it
    <!-- code-check: micropython -->  force the target (or: cpython)

Exit status is 1 if any error was found, so the course agent and CI can gate
on it. Warnings never fail the run.
"""

from __future__ import annotations

import argparse
import ast
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import textwrap
from dataclasses import dataclass, field
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(Path(__file__).resolve().parent))
import uno_q_checks  # noqa: E402  (Arduino UNO Q: Bridge contract, sketch lint, main.py run)

PY_FENCES = {"python", "py", "python3", "micropython", "mpy", "upython"}

# Imports that only exist on a microcontroller: their presence makes a block
# MicroPython even in a course that isn't tagged that way.
MP_ONLY_MODULES = {
    "machine", "rp2", "utime", "network", "neopixel", "micropython", "ujson",
    "uos", "ubinascii", "urequests", "umqtt", "bluetooth", "aioble", "dht",
    "onewire", "ds18x20", "framebuf", "picozero",
}

# Modules on stock rp2 / Pico W firmware (1.2x) that the runner can execute.
MP_FIRMWARE_MODULES = {
    "array", "asyncio", "uasyncio", "binascii", "builtins", "cmath",
    "collections", "cryptolib", "deflate", "errno", "gc", "hashlib", "heapq",
    "io", "json", "math", "os", "platform", "random", "re", "select",
    "struct", "sys", "time", "vfs", "_thread",
    "machine", "rp2", "micropython", "framebuf", "neopixel", "dht",
    "onewire", "ds18x20", "uctypes", "bluetooth", "network", "socket", "ssl",
    "ntptime", "requests", "urequests", "mip", "umqtt", "aioble", "utime",
    "uos", "ujson", "ubinascii", "urandom", "ustruct", "ure", "uio",
    "lwip", "webrepl", "espnow", "cyw43",
}

# On the Pico's firmware but not in the MicroPython unix port, and not stubbed.
NOT_ON_UNIX_PORT = {"onewire", "ds18x20", "bluetooth", "cyw43", "aioble", "lwip"}

# Modules that need the network; blocks using them are parsed and API
# checked but not run.
NETWORK_MODULES = {
    "network", "socket", "ssl", "requests", "urequests", "umqtt", "ntptime",
    "mip", "aioble", "bluetooth", "webrepl", "espnow",
}

# CPython modules that are not in MicroPython firmware. Some can be added
# with `mip`, so they're a warning when the lesson mentions mip.
CPYTHON_ONLY = {
    "datetime": "use time.localtime() (a tuple) or install it with mip",
    "statistics": "work it out with sum()/len()",
    "threading": "use _thread or asyncio",
    "subprocess": None, "pathlib": "use os", "typing": None,
    "numpy": "use ulab on a custom firmware, or plain lists",
    "pandas": None, "csv": "write comma-separated lines yourself",
    "logging": "install it with mip, or use print()",
    "argparse": None, "dataclasses": None, "enum": "use class constants",
    "functools": "install it with mip", "itertools": "install it with mip",
    "string": None, "textwrap": None, "tkinter": None, "smbus": None,
    "smbus2": None, "RPi": "that's Raspberry Pi (Linux) code, not the Pico",
    "gpiozero": "that's Raspberry Pi (Linux) code; picozero is the Pico one",
    "serial": "that's pyserial for a computer", "abc": None, "copy": None,
    "decimal": None, "fractions": None, "shutil": None, "glob": None,
    "queue": None, "signal": None, "unittest": "use mip's unittest",
}

# What the rp2 firmware's modules really expose. Anything else is flagged.
MP_MODULE_API = {
    "time": {
        "sleep", "sleep_ms", "sleep_us", "ticks_ms", "ticks_us", "ticks_cpu",
        "ticks_add", "ticks_diff", "time", "time_ns", "localtime", "gmtime",
        "mktime",
    },
    "machine": {
        "Pin", "Signal", "ADC", "PWM", "I2C", "SoftI2C", "SPI", "SoftSPI",
        "UART", "Timer", "RTC", "WDT", "I2S", "USBDevice", "reset",
        "soft_reset", "reset_cause", "bootloader", "freq", "idle",
        "lightsleep", "deepsleep", "unique_id", "time_pulse_us", "bitstream",
        "disable_irq", "enable_irq", "mem8", "mem16", "mem32",
        "PWRON_RESET", "WDT_RESET",
    },
    "os": {
        "listdir", "ilistdir", "mkdir", "remove", "rename", "rmdir", "stat",
        "statvfs", "uname", "urandom", "getcwd", "chdir", "sync", "dupterm",
        "mount", "umount", "VfsFat", "VfsLfs2", "sep",
    },
}
MP_MODULE_API["utime"] = MP_MODULE_API["time"]
MP_MODULE_API["uos"] = MP_MODULE_API["os"]

# Hints for the most common slips, keyed by "module.attr".
MP_HINTS = {
    "time.strftime": "format the tuple from time.localtime() yourself",
    "time.perf_counter": "use time.ticks_us() and time.ticks_diff()",
    "time.monotonic": "use time.ticks_ms() and time.ticks_diff()",
    "time.struct_time": "time.localtime() returns a plain tuple",
    "time.ctime": "format the tuple from time.localtime() yourself",
    "time.asctime": "format the tuple from time.localtime() yourself",
}

LOCALTIME_HINT = (
    "on MicroPython time.localtime() returns a plain tuple "
    "(year, month, mday, hour, minute, second, weekday, yearday), so "
    "`.{attr}` fails with AttributeError: index it instead, "
    "e.g. t = time.localtime(); hour = t[3]"
)


@dataclass
class Block:
    path: Path
    line: int  # 1-based line of the first code line in the markdown file
    code: str
    target: str  # "micropython" | "cpython"
    run: bool = True
    repl: bool = False


@dataclass
class Finding:
    path: Path
    line: int
    level: str  # "error" | "warning"
    rule: str
    message: str

    def fmt(self) -> str:
        try:
            rel = self.path.resolve().relative_to(REPO)
        except ValueError:
            rel = self.path
        return f"{rel}:{self.line}: {self.level} [{self.rule}] {self.message}"


@dataclass
class Report:
    findings: list = field(default_factory=list)
    blocks: int = 0
    ran: int = 0

    def add(self, block_or_path, line, level, rule, message):
        path = getattr(block_or_path, "path", block_or_path)
        self.findings.append(Finding(path, line, level, rule, message))


# ---------------------------------------------------------------------------
# Extracting code from a lesson
# ---------------------------------------------------------------------------

FENCE_RE = re.compile(r"^(\s*(?:>\s*)?)(`{3,}|~{3,})\s*([\w+-]*)")
MARKER_RE = re.compile(r"<!--\s*code-check:\s*([\w-]+)\s*-->")
CAPTURE_RE = re.compile(r"{%-?\s*capture\s+(\w+)\s*-?%}")
ENDCAPTURE_RE = re.compile(r"{%-?\s*endcapture\s*-?%}")


def course_groups(lesson: Path) -> set:
    yml = lesson.parent / "course.yml"
    if not yml.exists():
        return set()
    groups, in_groups = set(), False
    for raw in yml.read_text(encoding="utf-8", errors="replace").splitlines():
        s = raw.strip()
        if s.startswith("groups:"):
            in_groups = True
            continue
        if in_groups:
            if s.startswith("- "):
                groups.add(s[2:].strip().lower())
            elif s:
                in_groups = False
    return groups


def strip_repl(code: str) -> str:
    """Keep only the input lines of a `>>>` REPL transcript."""
    out = []
    for line in code.splitlines():
        if line.startswith(">>> ") or line == ">>>":
            out.append(line[4:])
        elif line.startswith("... "):
            out.append(line[4:])
        elif line == "...":
            out.append("")
    return "\n".join(out)


def imported_modules(tree: ast.AST) -> list:
    mods = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for a in node.names:
                mods.append((a.name.split(".")[0], node.lineno))
        elif isinstance(node, ast.ImportFrom) and node.module and node.level == 0:
            mods.append((node.module.split(".")[0], node.lineno))
    return mods


def guess_target(code: str, fence: str, groups: set, override: str | None) -> str:
    if override in ("micropython", "cpython"):
        return override
    if "snakie-plugin" in groups or re.search(r"^\s*from\s+snakie\s+import\b", code, re.M):
        return "cpython"  # Snakie desktop plugins run on CPython, not on the board
    if fence in ("micropython", "mpy", "upython"):
        return "micropython"
    if re.search(r"^\s*(?:import|from)\s+(" + "|".join(MP_ONLY_MODULES) + r")\b", code, re.M):
        return "micropython"
    if groups & {"micropython", "pico"}:
        # A host-side script in a Pico course (pyserial, Flask, ...) is CPython.
        if re.search(r"^\s*(?:import|from)\s+(serial|flask|fastapi|tkinter|pandas|numpy|requests)\b", code, re.M) \
                and "machine" not in code:
            return "cpython"
        return "micropython"
    return "cpython"


def extract_blocks(path: Path) -> list:
    text = path.read_text(encoding="utf-8", errors="replace")
    lines = text.splitlines()
    groups = course_groups(path)
    if re.search(r"^\s*from\s+snakie\s+import\b", text, re.M):
        groups = groups | {"snakie-plugin"}  # the whole lesson is plugin code
    # `code_target: cpython` (or micropython) in a lesson's front matter, or in
    # course.yml, sets the default for a course that mixes computer and Pico code.
    fm = re.match(r"^---\n(.*?)\n---", text, re.S)
    target_default = None
    for src in (fm.group(1) if fm else "", _course_yml_text(path)):
        t = re.search(r"^\s*code_target:\s*(\w+)", src, re.M)
        if t:
            target_default = t.group(1).lower()
            break
    blocks = []
    pending = set()
    i = 0
    while i < len(lines):
        line = lines[i]
        m_marker = MARKER_RE.findall(line)
        if m_marker:
            pending.update(m_marker)
            i += 1
            continue

        m_cap = CAPTURE_RE.search(line)
        if m_cap and "code" in m_cap.group(1):
            first = line[m_cap.end():]
            start = i if first.strip() else i + 1
            j = i
            if not ENDCAPTURE_RE.search(first):
                j = i + 1
                while j < len(lines) and not ENDCAPTURE_RE.search(lines[j]):
                    j += 1
            body = ([first] if first.strip() else []) + lines[i + 1:j]
            last = lines[j] if j < len(lines) else ""
            if j > i:
                tail = ENDCAPTURE_RE.split(last)[0]
                if tail.strip():
                    body.append(tail)
            else:
                body = [ENDCAPTURE_RE.split(first)[0]]
            blocks.append(_make_block(path, start + 1, "\n".join(body), "python", groups,
                                      pending, target_default))
            pending = set()
            i = j + 1
            continue

        m = FENCE_RE.match(line)
        if m:
            indent, fence, lang = m.group(1), m.group(2), m.group(3).lower()
            start = i + 1
            j = start
            while j < len(lines):
                stripped = lines[j].lstrip(" \t>")
                if stripped.startswith(fence[0] * 3) and stripped.strip().strip(fence[0]) == "":
                    break
                j += 1
            if lang in PY_FENCES:
                body = []
                for raw in lines[start:j]:
                    body.append(raw[len(indent):] if raw.startswith(indent) else raw.lstrip(" >"))
                code = textwrap.dedent("\n".join(body))
                # A kramdown IAL such as {: .no-snakie } under the fence marks
                # an illustrative snippet, same as <!-- code-check: fragment -->.
                after = lines[j + 1].strip().lstrip("> ") if j + 1 < len(lines) else ""
                marks = set(pending)
                if after.startswith("{:") and "no-snakie" in after:
                    marks.add("fragment")
                blocks.append(_make_block(path, start + 1, code, lang, groups, marks, target_default))
            pending = set()
            i = j + 1
            continue

        if line.strip():
            pending = set()
        i += 1
    return [b for b in blocks if b is not None]


LIQUID_RAW_RE = re.compile(r"{%-?\s*(?:end)?raw\s*-?%}")


def _course_yml_text(lesson: Path) -> str:
    yml = lesson.parent / "course.yml"
    return yml.read_text(encoding="utf-8", errors="replace") if yml.exists() else ""


def _make_block(path, line, code, fence, groups, markers, target_default=None):
    code = LIQUID_RAW_RE.sub("", code)
    # Notebook magics (%pip, !ls) aren't Python; keep the line count.
    code = re.sub(r"^\s*[%!].*$", "", code, flags=re.M)
    if "skip" in markers or not code.strip():
        return None
    repl = bool(re.search(r"^>>> ", code, re.M))
    if repl:
        code = strip_repl(code)
    override = next((m for m in markers if m in ("micropython", "cpython")), target_default)
    target = guess_target(code, fence, groups, override)
    return Block(path, line, code, target, run=("fragment" not in markers) and not repl, repl=repl)


# ---------------------------------------------------------------------------
# Static checks
# ---------------------------------------------------------------------------

class MicroPythonApiVisitor(ast.NodeVisitor):
    def __init__(self, block: Block, report: Report, lesson_text: str):
        self.block = block
        self.report = report
        self.lesson_text = lesson_text
        self.aliases = {}  # local name -> real module name

    def err(self, node, rule, msg, level="error"):
        self.report.add(self.block, self.block.line + node.lineno - 1, level, rule, msg)

    def visit_Import(self, node):
        for a in node.names:
            top = a.name.split(".")[0]
            self.aliases[a.asname or top] = a.name if a.asname else top
            self._check_module(node, top)
        self.generic_visit(node)

    def visit_ImportFrom(self, node):
        if node.module and node.level == 0:
            top = node.module.split(".")[0]
            self._check_module(node, top)
            api = MP_MODULE_API.get(node.module)
            if api:
                for a in node.names:
                    if a.name != "*" and a.name not in api:
                        self._unknown_attr(node, node.module, a.name)
        self.generic_visit(node)

    def _check_module(self, node, top):
        if top in CPYTHON_ONLY:
            hint = CPYTHON_ONLY[top]
            mip = "mip" in self.lesson_text
            level = "warning" if mip else "error"
            msg = f"`{top}` isn't part of MicroPython firmware"
            if hint:
                msg += f": {hint}"
            self.err(node, "micropython-import", msg, level)

    def _unknown_attr(self, node, module, attr):
        real = "time" if module == "utime" else ("os" if module == "uos" else module)
        hint = MP_HINTS.get(f"{real}.{attr}")
        msg = f"`{module}.{attr}` doesn't exist in MicroPython on the Pico"
        if hint:
            msg += f": {hint}"
        self.err(node, "micropython-api", msg)

    def visit_Attribute(self, node):
        if node.attr.startswith("tm_"):
            self.err(node, "micropython-localtime", LOCALTIME_HINT.format(attr="tm_*"))
        if isinstance(node.value, ast.Name):
            module = self.aliases.get(node.value.id)
            api = MP_MODULE_API.get(module or "")
            if api and node.attr not in api and not node.attr.startswith("__"):
                self._unknown_attr(node, module, node.attr)
        self.generic_visit(node)

    def visit_Call(self, node):
        # print(..., flush=True) is CPython only.
        if isinstance(node.func, ast.Name) and node.func.id == "print":
            for kw in node.keywords:
                if kw.arg == "flush":
                    self.err(node, "micropython-api",
                             "MicroPython's print() has no `flush=` argument")
        self.generic_visit(node)


def static_checks(block: Block, report: Report, lesson_text: str) -> ast.AST | None:
    try:
        tree = ast.parse(block.code)
    except SyntaxError as e:
        line = block.line + (e.lineno or 1) - 1
        bad = block.code.splitlines()[(e.lineno or 1) - 1] if e.lineno else ""
        if "..." in bad or "\u2026" in bad:
            report.add(block, line, "warning", "syntax",
                       f"not valid Python ({e.msg}), looks like an illustrative snippet with `...`: "
                       "use a ```text fence or mark it <!-- code-check: skip -->")
        else:
            report.add(block, line, "error", "syntax", f"not valid Python: {e.msg}")
        return None
    if block.target == "micropython":
        MicroPythonApiVisitor(block, report, lesson_text).visit(tree)
    return tree


# ---------------------------------------------------------------------------
# Running MicroPython blocks
# ---------------------------------------------------------------------------

def find_micropython(arg: str | None) -> str | None:
    cached = Path.home() / ".cache" / "kevsrobots" / "micropython" / "micropython"
    for cand in (arg, os.environ.get("MICROPYTHON"), shutil.which("micropython"), str(cached)):
        if cand and Path(cand).expanduser().exists():
            return str(Path(cand).expanduser())
    return None


STUBS = (Path(__file__).resolve().parent / "course_code_stubs.py")

# Exceptions that mean the code itself is wrong for MicroPython, whatever
# hardware is attached. Anything else (OSError, RuntimeError, IndexError...)
# may just be the runner's blank sensors, so it's a warning.
DEFINITE_RUN_ERRORS = {"AttributeError", "ImportError", "TypeError", "SyntaxError",
                       "NotImplementedError", "UnicodeError"}

RESULT_RE = re.compile(r"^@@CHECK (\w+) (.*)$", re.M)


def run_block(mp: str, block: Block, tree: ast.AST, defined_before: set,
              course_modules: set, report: Report, timeout: float) -> None:
    mods = {m for m, _ in imported_modules(tree)}
    if mods & (NETWORK_MODULES | NOT_ON_UNIX_PORT):
        return  # needs Wi-Fi or a driver the runner lacks: parse and API check only
    unknown = sorted(m for m in mods if m not in MP_FIRMWARE_MODULES and m not in CPYTHON_ONLY)
    if unknown:
        local = [m for m in unknown if m in course_modules]
        external = [m for m in unknown if m not in course_modules]
        if external:
            first = next(l for m, l in imported_modules(tree) if m == external[0])
            report.add(block, block.line + first - 1, "warning", "external-module",
                       f"needs `{'`, `'.join(external)}`, which isn't on stock firmware: "
                       "make sure the lesson says where to get it (not run)")
        return
    if mods & set(CPYTHON_ONLY):
        return  # already reported statically

    with tempfile.TemporaryDirectory() as tmp:
        src = Path(tmp) / "lesson_block.py"
        src.write_text(block.code, encoding="utf-8")
        runner = Path(tmp) / "runner.py"
        runner.write_text(
            STUBS.read_text(encoding="utf-8")
            + f"\n_run_lesson_block({str(src)!r})\n",
            encoding="utf-8",
        )
        try:
            proc = subprocess.run([mp, str(runner)], capture_output=True, text=True,
                                  timeout=timeout, stdin=subprocess.DEVNULL, cwd=tmp)
            out = proc.stdout + proc.stderr
        except subprocess.TimeoutExpired:
            report.ran += 1
            return  # an endless loop without sleep: not an error in itself
    report.ran += 1

    m = RESULT_RE.search(out)
    if not m:
        tail = out.strip().splitlines()[-3:]
        report.add(block, block.line, "error", "run", "crashed: " + " / ".join(tail))
        return
    status, payload = m.group(1), m.group(2)
    if status in ("ok", "stopped"):
        return
    info = json.loads(payload)
    line = block.line + max(info.get("line", 1), 1) - 1
    exc, msg = info.get("type", "Error"), info.get("msg", "")
    if exc == "NameError":
        name = re.search(r"'(\w+)'", msg)
        if name and name.group(1) in defined_before:
            return  # continues an earlier block in this lesson
        what = f"`{name.group(1)}`" if name else "a name"
        report.add(block, line, "warning", "fragment",
                   f"uses {what}, which neither this block nor an earlier one in the lesson "
                   "defines. Fine for a snippet (mark it <!-- code-check: fragment -->), "
                   "a bug in a full program")
        return
    if exc == "ImportError" and any(f"'{m}'" in msg or m in msg for m in course_modules):
        return
    if exc == "SyntaxError" and "outside function" in msg:
        report.add(block, line, "warning", "fragment",
                   f"{msg}: a piece of a function body. Fine as a snippet "
                   "(mark it <!-- code-check: fragment -->)")
        return
    if exc not in DEFINITE_RUN_ERRORS:
        report.add(block, line, "warning", "run-hardware",
                   f"stopped with {exc}: {msg} while running against simulated hardware "
                   "(no I2C devices, blank sensor data, no files). Check it on a real board")
        return
    hint = ""
    tm = re.search(r"has no attribute '(tm_\w+)'", msg)
    if tm:
        hint = ": " + LOCALTIME_HINT.format(attr=tm.group(1))
    report.add(block, line, "error", "run",
               f"fails on MicroPython: {exc}: {msg}{hint}")


def names_defined(tree: ast.AST) -> set:
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            names.add(node.name)
        elif isinstance(node, ast.Name) and isinstance(node.ctx, ast.Store):
            names.add(node.id)
        elif isinstance(node, (ast.Import, ast.ImportFrom)):
            for a in node.names:
                names.add((a.asname or a.name).split(".")[0])
    return names


def course_module_names(lesson: Path) -> set:
    """Module names a course defines itself (e.g. `ssd1306.py` it tells you to save)."""
    names = set()
    for md in lesson.parent.glob("*.md"):
        txt = md.read_text(encoding="utf-8", errors="replace")
        names.update(re.findall(r"\b(\w+)\.py\b", txt))
    return names


# ---------------------------------------------------------------------------

def lesson_files(paths: list) -> list:
    out = []
    for p in paths:
        p = Path(p)
        if p.is_dir():
            out.extend(sorted(p.rglob("*.md")))
        elif p.suffix == ".md" and p.exists():
            out.append(p)
    return [p for p in out if "/assets/" not in str(p)]


def changed_lessons(base: str) -> list:
    res = subprocess.run(["git", "diff", "--name-only", "--diff-filter=ACMR", f"{base}...HEAD", "--", "source"],
                         capture_output=True, text=True, cwd=REPO, check=True)
    return [REPO / f for f in res.stdout.split() if f.endswith(".md")]


def check(paths: list, mp: str | None, timeout: float = 5.0, run_host: bool = True) -> Report:
    report = Report()
    for lesson in lesson_files(paths):
        text = lesson.read_text(encoding="utf-8", errors="replace")
        course_mods = course_module_names(lesson)
        defined = set()
        host_blocks = []
        for block in extract_blocks(lesson):
            report.blocks += 1
            tree = static_checks(block, report, text)
            if tree is None:
                continue
            if mp and block.run and block.target == "micropython":
                run_block(mp, block, tree, defined, course_mods, report, timeout)
            if block.target == "cpython":
                host_blocks.append((block, tree))
            defined |= names_defined(tree)
        uno_q_checks.check_lesson(lesson, text, host_blocks, report, run=run_host, timeout=timeout)
    return report


def dedupe(findings: list) -> list:
    """One finding per line and rule, and no run error where a static check
    already explained that line."""
    seen, static_lines, out = set(), set(), []
    for f in findings:
        if f.level == "error" and f.rule != "run":
            static_lines.add((f.path, f.line))
    for f in findings:
        key = (f.path, f.line, f.rule)
        if key in seen:
            continue
        if f.rule == "run" and (f.path, f.line) in static_lines:
            continue
        seen.add(key)
        out.append(f)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("paths", nargs="*", help="course folders or lesson .md files")
    ap.add_argument("--changed", metavar="BASE", help="check lessons changed since BASE (e.g. origin/main)")
    ap.add_argument("--micropython", help="path to a MicroPython unix binary (or set $MICROPYTHON)")
    ap.add_argument("--no-run", action="store_true", help="parse and API check only")
    ap.add_argument("--timeout", type=float, default=5.0, help="seconds per block run (default 5)")
    ap.add_argument("--github", action="store_true", help="print GitHub Actions annotations")
    ap.add_argument("--errors-only", action="store_true", help="hide warnings")
    args = ap.parse_args(argv)

    paths = list(args.paths)
    if args.changed:
        paths += changed_lessons(args.changed)
    if not paths:
        ap.error("give course folders/lessons, or --changed BASE")

    mp = None if args.no_run else find_micropython(args.micropython)
    if not args.no_run and not mp:
        print("note: no MicroPython binary found, so blocks were parsed and API checked "
              "but not run. Build one with scripts/get_micropython.sh (once, ~2 minutes)",
              file=sys.stderr)

    report = check(paths, mp, args.timeout, run_host=not args.no_run)
    report.findings = dedupe(report.findings)
    errors = [f for f in report.findings if f.level == "error"]
    shown = errors if args.errors_only else report.findings
    for f in sorted(shown, key=lambda f: (str(f.path), f.line)):
        print(f.fmt())
        if args.github:
            try:
                rel = f.path.resolve().relative_to(REPO)
            except ValueError:
                rel = f.path
            msg = f.message.replace("%", "%25").replace("\n", "%0A")
            print(f"::{f.level} file={rel},line={f.line},title={f.rule}::{msg}")
    warnings = len(report.findings) - len(errors)
    print(f"\n{report.blocks} code blocks checked, {report.ran} run: "
          f"{len(errors)} errors, {warnings} warnings")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
