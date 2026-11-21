"""Run one lesson block against the stand-in arduino package and report."""
import json
import sys
import traceback

from arduino.app_utils import StopLesson

path = sys.argv[1]
src = open(path, encoding="utf-8").read()
try:
    exec(compile(src, path, "exec"), {"__name__": "__main__"})
    print("@@CHECK ok {}")
except StopLesson:
    print("@@CHECK stopped {}")
except SystemExit:
    print("@@CHECK ok {}")
except BaseException as exc:  # noqa: BLE001
    line = 1
    for frame in traceback.extract_tb(exc.__traceback__):
        if frame.filename == path:
            line = frame.lineno
    if isinstance(exc, SyntaxError) and exc.lineno:
        line = exc.lineno
    print("@@CHECK error " + json.dumps({"type": type(exc).__name__, "msg": str(exc), "line": line}))
