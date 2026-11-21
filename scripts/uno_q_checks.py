"""Extra course-code checks for Arduino UNO Q lessons.

An UNO Q App is two programs: a Python script on the Linux side and an
Arduino sketch on the microcontroller, joined by the Bridge (RPC). The usual
checker only parses computer-side Python and never looks at C++, so a lesson
could ship a ``Bridge.call("set_led")`` that no sketch provides and still
pass. These checks run for any lesson that mentions ``arduino.app_utils`` or
``Arduino_RouterBridge``:

1. **Bridge contract.** Every name Python calls or notifies must be provided
   by a sketch block in the same lesson with the same number of arguments,
   and the other way round.
2. **Sketch lint.** Things Arduino's documentation says break on this board:
   talking over the Bridge or printing inside a ``Bridge.provide()`` callback
   (deadlock: use ``provide_safe``), the generic ``Servo`` library, opening
   ``Serial1`` or ``/dev/ttyHS1`` (the router owns them), ``Wire`` instead of
   ``Wire1`` for Qwiic, a sketch that uses the Bridge without ``Bridge.begin()``.
3. **Run.** Each full ``main.py`` is run on CPython against a strict stand-in
   for ``arduino.app_utils`` (scripts/uno_q_stubs). ``Bridge.call`` returns a
   value of the type the lesson's sketch function returns, and the user loop
   stops after a few passes.

Facts are from Arduino's Bridge API reference and the UNO Q user manual
(docs.arduino.cc/software/app-lab/bridge/bridge-api and
docs.arduino.cc/tutorials/uno-q/user-manual).

None of this proves the code works on a real board. It catches the mistakes
that can be caught without one.
"""

from __future__ import annotations

import ast
import json
import os
import re
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

STUB_DIR = Path(__file__).resolve().parent / "uno_q_stubs"

CPP_FENCES = {"cpp", "c++", "arduino", "ino", "c"}
TRIGGER_RE = re.compile(r"arduino\.app_utils|Arduino_RouterBridge|arduino\.app_bricks")
FENCE_RE = re.compile(r"^(\s*(?:>\s*)?)(`{3,}|~{3,})\s*([\w+-]*)")
MARKER_RE = re.compile(r"<!--\s*code-check:\s*([\w-]+)\s*-->")

# Bricks the stand-ins cover. A block importing any other Brick is checked
# statically but not run.
STUBBED_BRICKS: set = {"web_ui"}


@dataclass
class CppBlock:
    line: int  # 1-based line of the first code line
    code: str


BOARD_RE = re.compile(r"^\s*code_board:\s*uno[-_ ]?q\b", re.M | re.I)


def applies(text: str, course_yml: str = "") -> bool:
    """A lesson that uses the Bridge, or any lesson of a course with
    `code_board: uno-q` in its course.yml (so sketch-only lessons are linted too)."""
    return bool(TRIGGER_RE.search(text) or BOARD_RE.search(course_yml))


def extract_cpp_blocks(text: str) -> list:
    lines = text.splitlines()
    blocks, pending, i = [], set(), 0
    while i < len(lines):
        line = lines[i]
        marks = MARKER_RE.findall(line)
        if marks:
            pending.update(marks)
            i += 1
            continue
        m = FENCE_RE.match(line)
        if m:
            fence, lang = m.group(2), m.group(3).lower()
            j = i + 1
            while j < len(lines) and not lines[j].strip().startswith(fence):
                j += 1
            if lang in CPP_FENCES and "skip" not in pending:
                blocks.append(CppBlock(i + 2, "\n".join(lines[i + 1:j])))
            pending = set()
            i = j + 1
            continue
        if line.strip():
            pending = set()
        i += 1
    return blocks


# ---------------------------------------------------------------------------
# C++ side: a light scan, not a parser
# ---------------------------------------------------------------------------

def _strip_cpp_comments(code: str) -> str:
    """Blank out comments and string contents, keeping every line and column."""
    def blank(m):
        return re.sub(r"[^\n]", " ", m.group(0))
    code = re.sub(r"/\*.*?\*/", blank, code, flags=re.S)
    return re.sub(r"//[^\n]*", blank, code)


def _split_args(s: str) -> list:
    args, depth, cur, quote = [], 0, "", None
    for ch in s:
        if quote:
            cur += ch
            if ch == quote:
                quote = None
            continue
        if ch in "\"'":
            quote = ch
            cur += ch
        elif ch in "([{<":
            depth += 1
            cur += ch
        elif ch in ")]}>":
            depth -= 1
            cur += ch
        elif ch == "," and depth == 0:
            args.append(cur.strip())
            cur = ""
        else:
            cur += ch
    if cur.strip():
        args.append(cur.strip())
    return args


def _call_args(code: str, open_paren: int):
    """Return (argument text, index after the closing paren)."""
    depth, quote = 0, None
    for k in range(open_paren, len(code)):
        ch = code[k]
        if quote:
            if ch == quote and code[k - 1] != "\\":
                quote = None
            continue
        if ch in "\"'":
            quote = ch
        elif ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
            if depth == 0:
                return code[open_paren + 1:k], k + 1
    return code[open_paren + 1:], len(code)


CPP_FUNC_RE = re.compile(
    r"^[ \t]*((?:(?:unsigned|const|static|inline)\s+)*[A-Za-z_][\w:<>]*[\s\*&]+)"
    r"([A-Za-z_]\w*)\s*\(([^;{}()]*)\)\s*\{", re.M)
CPP_KEYWORDS = {"if", "for", "while", "switch", "return", "else", "do"}


@dataclass
class CppFunc:
    name: str
    ret: str
    nparams: int
    body: str
    line: int  # line within the block, 1-based


def cpp_functions(code: str) -> dict:
    clean = _strip_cpp_comments(code)
    funcs = {}
    for m in CPP_FUNC_RE.finditer(clean):
        name = m.group(2)
        if name in CPP_KEYWORDS:
            continue
        params = m.group(3).strip()
        n = 0 if params in ("", "void") else len(_split_args(params))
        depth, k = 0, m.end() - 1
        while k < len(clean):
            if clean[k] == "{":
                depth += 1
            elif clean[k] == "}":
                depth -= 1
                if depth == 0:
                    break
            k += 1
        funcs[name] = CppFunc(name, " ".join(m.group(1).split()), n,
                              clean[m.end():k], clean.count("\n", 0, m.start()) + 1)
    return funcs


CPP_BRIDGE_RE = re.compile(r"\bBridge\s*\.\s*(provide_safe|provide|call|notify)\s*\(")


@dataclass
class BridgeUse:
    kind: str      # provide | provide_safe | call | notify
    name: str      # RPC name
    nargs: int     # arguments after the name (call/notify)
    handler: str   # function registered (provide), "" for a lambda
    line: int      # 1-based line in the markdown file


def cpp_bridge_uses(block: CppBlock) -> list:
    clean = _strip_cpp_comments(block.code)
    uses = []
    for m in CPP_BRIDGE_RE.finditer(clean):
        argtext, _ = _call_args(block.code, m.end() - 1)
        args = _split_args(argtext)
        if not args:
            continue
        nm = re.match(r'^"([^"]*)"$', args[0])
        if not nm:
            continue  # a name built at run time: can't check it
        line = block.line + clean.count("\n", 0, m.start())
        kind = m.group(1)
        handler = ""
        if kind.startswith("provide") and len(args) > 1 and re.match(r"^[A-Za-z_]\w*$", args[1]):
            handler = args[1]
        uses.append(BridgeUse(kind, nm.group(1), len(args) - 1, handler, line))
    return uses


# ---------------------------------------------------------------------------
# Python side
# ---------------------------------------------------------------------------

def py_bridge_uses(block, tree: ast.AST) -> list:
    uses = []
    funcs = {n.name: n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)}

    def arity(fn: ast.FunctionDef):
        a = fn.args
        if a.vararg:
            return None
        return len(a.posonlyargs) + len(a.args)

    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) \
                and isinstance(node.func.value, ast.Name) and node.func.value.id == "Bridge" \
                and node.func.attr in ("call", "notify", "provide") and node.args \
                and isinstance(node.args[0], ast.Constant) and isinstance(node.args[0].value, str):
            line = block.line + node.lineno - 1
            name = node.args[0].value
            if node.func.attr == "provide":
                n = None
                if len(node.args) > 1 and isinstance(node.args[1], ast.Name) and node.args[1].id in funcs:
                    n = arity(funcs[node.args[1].id])
                uses.append(BridgeUse("provide", name, -1 if n is None else n, "", line))
            else:
                starred = any(isinstance(a, ast.Starred) for a in node.args)
                uses.append(BridgeUse(node.func.attr, name, -1 if starred else len(node.args) - 1, "", line))
        if isinstance(node, ast.FunctionDef):
            for dec in node.decorator_list:
                call = dec if isinstance(dec, ast.Call) else None
                fn = call.func if call else dec
                if not (isinstance(fn, ast.Name) and fn.id in ("call", "notify", "provide")):
                    continue
                name = node.name
                if call and call.args and isinstance(call.args[0], ast.Constant):
                    name = call.args[0].value
                elif call:
                    for kw in call.keywords:
                        if kw.arg == "method_name" and isinstance(kw.value, ast.Constant):
                            name = kw.value.value
                n = arity(node)
                uses.append(BridgeUse(fn.id, name, -1 if n is None else n, "",
                                      block.line + node.lineno - 1))
    return uses


# ---------------------------------------------------------------------------
# The checks
# ---------------------------------------------------------------------------

DEADLOCK_RE = re.compile(r"\b(Bridge\s*\.\s*(?:call|notify)|Monitor\s*\.\s*print(?:ln)?|Serial\s*\.\s*print(?:ln)?)\s*\(")


def lint_sketch(path: Path, block: CppBlock, lesson_text: str, report) -> None:
    clean = _strip_cpp_comments(block.code)
    funcs = cpp_functions(block.code)
    uses = cpp_bridge_uses(block)

    def line_of(pattern):
        m = re.search(pattern, clean)
        return block.line + clean.count("\n", 0, m.start()) if m else None

    for u in uses:
        if u.kind == "provide" and u.handler in funcs:
            f = funcs[u.handler]
            bad = DEADLOCK_RE.search(f.body)
            if bad:
                what = re.sub(r"\s+", "", bad.group(1))
                report.add(path, block.line + f.line - 1, "error", "bridge-deadlock",
                           f"`{f.name}` is registered with Bridge.provide() and uses `{what}()`. "
                           "A provide() callback runs in the Bridge's own thread, so starting "
                           "another message from inside it deadlocks the board. Register it with "
                           "Bridge.provide_safe() so it runs in loop()")

    ln = line_of(r"#\s*include\s*<Servo\.h>")
    if ln:
        report.add(path, ln, "error", "uno-q-servo",
                   "the generic Servo library isn't the one for the UNO Q: use "
                   "Arduino_HardwareServo (`#include <Arduino_HardwareServo.h>`), which drives "
                   "servos from the STM32's hardware timers")
    ln = line_of(r"\bSerial1\s*\.")
    if ln:
        report.add(path, ln, "error", "uno-q-reserved",
                   "Serial1 is the link to the Linux side and belongs to the arduino-router "
                   "service. Opening it breaks the Bridge")
    if re.search(r"\bBridge\s*\.\s*(provide|provide_safe|call|notify)\s*\(", clean) \
            and re.search(r"\bvoid\s+setup\s*\(", clean) \
            and not re.search(r"\bBridge\s*\.\s*begin\s*\(", clean):
        report.add(path, line_of(r"\bBridge\s*\.") or block.line, "error", "bridge-begin",
                   "the sketch uses the Bridge but never calls Bridge.begin() in setup()")
    if re.search(r"\b(Qwiic|Modulino)\b", lesson_text) and re.search(r"\bWire\s*\.\s*begin\s*\(", clean) \
            and "Wire1" not in clean:
        report.add(path, line_of(r"\bWire\s*\.\s*begin") or block.line, "warning", "uno-q-qwiic",
                   "the Qwiic connector is on the second I2C bus: use Wire1, not Wire "
                   "(Wire is the A4/A5 header pins)")


def lint_python(block, report) -> None:
    for n, line in enumerate(block.code.splitlines()):
        if "/dev/ttyHS1" in line:
            report.add(block, block.line + n, "error", "uno-q-reserved",
                       "/dev/ttyHS1 is the link to the microcontroller and belongs to the "
                       "arduino-router service. Opening it breaks the Bridge")


def check_contract(path: Path, py_uses: list, cpp_uses: list, cpp_funcs: dict, report) -> None:
    sketch_provides = {}
    for u in cpp_uses:
        if u.kind.startswith("provide"):
            sketch_provides.setdefault(u.name, []).append(u)
    py_provides = {}
    for u in py_uses:
        if u.kind == "provide":
            py_provides.setdefault(u.name, []).append(u)
    have_sketch = bool(cpp_uses) or bool(cpp_funcs)
    have_python = bool(py_uses)

    for u in py_uses:
        if u.kind not in ("call", "notify") or not have_sketch:
            continue
        if u.name not in sketch_provides:
            report.add(path, u.line, "error", "bridge-contract",
                       f'Python uses Bridge.{u.kind}("{u.name}") but no sketch in this lesson '
                       f'provides "{u.name}". On the board this raises ValueError (call) or is '
                       "silently dropped (notify)")
            continue
        if u.nargs < 0:
            continue
        counts = {cpp_funcs[p.handler].nparams for p in sketch_provides[u.name] if p.handler in cpp_funcs}
        if counts and u.nargs not in counts:
            want = sorted(counts)[0]
            report.add(path, u.line, "error", "bridge-contract",
                       f'Bridge.{u.kind}("{u.name}") passes {u.nargs} argument(s) but the '
                       f"sketch function takes {want}")

    for u in cpp_uses:
        if u.kind not in ("call", "notify") or not have_python:
            continue
        if u.name not in py_provides:
            report.add(path, u.line, "error", "bridge-contract",
                       f'the sketch uses Bridge.{u.kind}("{u.name}") but no Python block in this '
                       f'lesson provides "{u.name}"')
            continue
        counts = {p.nargs for p in py_provides[u.name] if p.nargs >= 0}
        if counts and u.nargs not in counts:
            report.add(path, u.line, "error", "bridge-contract",
                       f'the sketch passes {u.nargs} argument(s) to "{u.name}" but the Python '
                       f"function takes {sorted(counts)[0]}")


CPP_RETURN_DEFAULTS = [
    (r"\bvoid\b", None), (r"\bbool\b", False), (r"\b(float|double)\b", 0.0),
    (r"\b(String|char)\b", ""), (r"\b(std::vector|std::array)\b", []),
    (r"\b(u?int\d*_t|int|long|short|unsigned|size_t|byte)\b", 0),
]


def return_values(cpp_uses: list, cpp_funcs: dict) -> dict:
    out = {}
    for u in cpp_uses:
        if not u.kind.startswith("provide"):
            continue
        value = None
        f = cpp_funcs.get(u.handler)
        if f:
            for pat, v in CPP_RETURN_DEFAULTS:
                if re.search(pat, f.ret):
                    value = v
                    break
        out[u.name] = value
    return out


RESULT_RE = re.compile(r"^@@CHECK (\w+) (.*)$", re.M)
STDLIB = set(getattr(sys, "stdlib_module_names", ()))


def run_python(block, tree, provided: dict, defined_before: set, report, timeout: float,
               sketch_calls: dict | None = None) -> bool:
    """Run one main.py block against the stand-in arduino package. Returns True if run."""
    mods = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            mods.update(a.name for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            mods.add(node.module)
    if not any(m.startswith("arduino.app_utils") for m in mods):
        return False  # a snippet, or plain Python: nothing for this runner to add
    for m in sorted(mods):
        if m.startswith("arduino.app_bricks"):
            brick = m.split(".")[2] if m.count(".") >= 2 else ""
            if brick not in STUBBED_BRICKS:
                return False
        elif not m.startswith("arduino") and m.split(".")[0] not in STDLIB:
            return False  # needs a package the checker doesn't have

    with tempfile.TemporaryDirectory() as tmp:
        src = Path(tmp) / "main.py"
        src.write_text(block.code, encoding="utf-8")
        env = dict(os.environ, PYTHONPATH=str(STUB_DIR), UNO_Q_PROVIDED=json.dumps(provided),
                   UNO_Q_SKETCH_CALLS=json.dumps(sketch_calls or {}))
        try:
            proc = subprocess.run([sys.executable, str(STUB_DIR / "_runner.py"), str(src)],
                                  capture_output=True, text=True, timeout=timeout,
                                  stdin=subprocess.DEVNULL, cwd=tmp, env=env)
        except subprocess.TimeoutExpired:
            report.add(block, block.line, "warning", "run-hardware",
                       "didn't finish in the checker: a loop that never sleeps or calls the Bridge?")
            return True
    m = RESULT_RE.search(proc.stdout + proc.stderr)
    if not m:
        tail = (proc.stdout + proc.stderr).strip().splitlines()[-3:]
        report.add(block, block.line, "error", "run", "crashed: " + " / ".join(tail))
        return True
    if m.group(1) in ("ok", "stopped"):
        return True
    info = json.loads(m.group(2))
    line = block.line + max(info.get("line", 1), 1) - 1
    exc, msg = info.get("type", "Error"), info.get("msg", "")
    if exc == "NameError":
        name = re.search(r"'(\w+)'", msg)
        if name and name.group(1) in defined_before:
            return True
        report.add(block, line, "warning", "fragment",
                   f"uses {'`' + name.group(1) + '`' if name else 'a name'}, which nothing in the "
                   "lesson defines. Fine for a snippet (mark it <!-- code-check: fragment -->), "
                   "a bug in a full program")
        return True
    report.add(block, line, "error", "run", f"fails with the stand-in UNO Q: {exc}: {msg}")
    return True


def check_lesson(path: Path, text: str, py_blocks: list, report, run: bool = True,
                 timeout: float = 5.0) -> None:
    """py_blocks: (Block, ast tree) pairs for the lesson's CPython blocks."""
    yml = path.parent / "course.yml"
    if not applies(text, yml.read_text(encoding="utf-8", errors="replace") if yml.exists() else ""):
        return
    cpp_blocks = extract_cpp_blocks(text)
    cpp_uses, cpp_funcs = [], {}
    for cb in cpp_blocks:
        report.blocks += 1
        lint_sketch(path, cb, text, report)
        cpp_uses += cpp_bridge_uses(cb)
        cpp_funcs.update(cpp_functions(cb.code))
    py_uses = []
    for block, tree in py_blocks:
        lint_python(block, report)
        py_uses += py_bridge_uses(block, tree)
    check_contract(path, py_uses, cpp_uses, cpp_funcs, report)

    if not run:
        return
    provided = return_values(cpp_uses, cpp_funcs)
    sketch_calls = {u.name: u.nargs for u in cpp_uses if u.kind in ("call", "notify")}
    defined = set()
    for block, tree in py_blocks:
        if block.run and run_python(block, tree, provided, defined, report, timeout, sketch_calls):
            report.ran += 1
        for node in ast.walk(tree):
            if isinstance(node, (ast.FunctionDef, ast.ClassDef)):
                defined.add(node.name)
            elif isinstance(node, ast.Name) and isinstance(node.ctx, ast.Store):
                defined.add(node.id)
