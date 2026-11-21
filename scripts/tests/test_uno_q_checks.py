"""Tests for scripts/uno_q_checks.py. Run: python3 -m pytest scripts/tests"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import check_course_code as ccc  # noqa: E402

# The LED example from Arduino's UNO Q user manual: the known-good case.
GOOD_PY = '''from arduino.app_utils import *
import time

led_state = False

def loop():
    global led_state
    time.sleep(1)
    led_state = not led_state
    Bridge.call("set_led_state", led_state)

App.run(user_loop=loop)
'''
GOOD_INO = '''#include "Arduino_RouterBridge.h"

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    Bridge.begin();
    Bridge.provide("set_led_state", set_led_state);
}

void loop() {
}

void set_led_state(bool state) {
    // LOW state means LED is ON
    digitalWrite(LED_BUILTIN, state ? LOW : HIGH);
}
'''


def lesson(tmp_path, py, ino, extra=""):
    (tmp_path / "course.yml").write_text("- name: T\n  groups:\n    - arduino\n")
    (tmp_path / "01.md").write_text(f"{extra}\n```python\n{py}```\n\n```cpp\n{ino}```\n")
    return ccc.check([tmp_path], mp=None)


def rules(report):
    return sorted({f.rule for f in report.findings})


def test_arduino_example_passes_and_runs(tmp_path):
    report = lesson(tmp_path, GOOD_PY, GOOD_INO)
    assert report.findings == []
    assert report.blocks == 2 and report.ran == 1


def test_name_nobody_provides(tmp_path):
    report = lesson(tmp_path, GOOD_PY.replace('"set_led_state"', '"set_led"'), GOOD_INO)
    assert "bridge-contract" in rules(report)
    assert all(f.level == "error" for f in report.findings)


def test_wrong_argument_count(tmp_path):
    report = lesson(tmp_path, GOOD_PY.replace("led_state)", "led_state, 5)"), GOOD_INO)
    assert [f.rule for f in report.findings] == ["bridge-contract"]
    assert "passes 2 argument(s)" in report.findings[0].message


def test_deadlock_in_provide_callback(tmp_path):
    ino = GOOD_INO.replace("digitalWrite(", 'Serial.println("led");\n    digitalWrite(')
    report = lesson(tmp_path, GOOD_PY, ino)
    assert rules(report) == ["bridge-deadlock"]
    # the same callback is fine once it runs in loop()
    safe = ino.replace("Bridge.provide(", "Bridge.provide_safe(")
    assert lesson(tmp_path, GOOD_PY, safe).findings == []


def test_commented_out_code_is_ignored(tmp_path):
    ino = GOOD_INO.replace("// LOW state means LED is ON", '// Serial.println("x"); Bridge.call("nope");')
    assert lesson(tmp_path, GOOD_PY, ino).findings == []


def test_sketch_calls_python(tmp_path):
    py = ('from arduino.app_utils import *\n\n'
          'def get_limit(room):\n    return 25\n\n'
          'Bridge.provide("get_limit", get_limit)\nApp.run()\n')
    ino = ('#include "Arduino_RouterBridge.h"\nvoid setup() {\n  Bridge.begin();\n}\n'
           'void loop() {\n  int limit;\n  if (Bridge.call("get_limit", "kitchen").result(limit)) {\n'
           '    Serial.println(limit);\n  }\n  delay(1000);\n}\n')
    assert lesson(tmp_path, py, ino).findings == []
    bad = lesson(tmp_path, py, ino.replace('"get_limit", "kitchen"', '"get_limit"'))
    assert "takes 1" in bad.findings[0].message
    missing = lesson(tmp_path, py.replace('"get_limit", get_limit', '"limit", get_limit'), ino)
    assert rules(missing) == ["bridge-contract"]


def test_board_specific_lint(tmp_path):
    ino = GOOD_INO.replace('#include "Arduino_RouterBridge.h"',
                           '#include "Arduino_RouterBridge.h"\n#include <Servo.h>\n#include <Wire.h>')
    ino = ino.replace("Bridge.begin();", "Wire.begin();\n    Serial1.begin(9600);")
    report = lesson(tmp_path, GOOD_PY, ino, extra="Plug the Modulino into the Qwiic connector.")
    assert rules(report) == ["bridge-begin", "uno-q-qwiic", "uno-q-reserved", "uno-q-servo"]


def test_python_that_would_fail_on_the_board(tmp_path):
    # Bridge.send() doesn't exist
    report = lesson(tmp_path, GOOD_PY.replace("Bridge.call(", "Bridge.send("), GOOD_INO)
    assert [(f.rule, f.level) for f in report.findings] == [("run", "error")]
    assert "send" in report.findings[0].message


def test_decorators(tmp_path):
    py = ('from arduino.app_utils import App, call, notify\n\n'
          '@call(timeout=3)\ndef read_light() -> int: ...\n\n'
          '@notify("set_led_state")\ndef set_led(state: bool): ...\n\n'
          'def loop():\n    set_led(read_light() > 500)\n\nApp.run(user_loop=loop)\n')
    ino = GOOD_INO.replace('Bridge.provide("set_led_state", set_led_state);',
                           'Bridge.provide("set_led_state", set_led_state);\n'
                           '    Bridge.provide_safe("read_light", read_light);')
    ino += "\nint read_light() {\n    return analogRead(A0);\n}\n"
    assert lesson(tmp_path, py, ino).findings == []
    assert "bridge-contract" in rules(lesson(tmp_path, py, GOOD_INO))


def test_lessons_without_the_bridge_are_untouched(tmp_path):
    (tmp_path / "course.yml").write_text("- name: T\n  groups:\n    - arduino\n")
    (tmp_path / "01.md").write_text("```cpp\n#include <Servo.h>\nvoid setup() { Serial1.begin(9600); }\n```\n")
    report = ccc.check([tmp_path], mp=None)
    assert report.findings == [] and report.blocks == 0


WEB_PY = '''from arduino.app_utils import *
from arduino.app_bricks.web_ui import WebUI

led_is_on = False

def toggle_led_state(client, data):
    global led_is_on
    led_is_on = not led_is_on
    Bridge.call("set_led_state", led_is_on)
    ui.send_message('led_status_update', {"led_is_on": led_is_on})

ui = WebUI()
ui.on_message('toggle_led', toggle_led_state)
App.run()
'''


def test_web_ui_brick_runs_its_handlers(tmp_path):
    # Arduino's "Blink LED with UI" example
    report = lesson(tmp_path, WEB_PY, GOOD_INO)
    assert report.findings == [] and report.ran == 1
    # ui.emit() isn't a WebUI method
    bad = lesson(tmp_path, WEB_PY.replace("ui.send_message(", "ui.emit("), GOOD_INO)
    assert [(f.rule, f.level) for f in bad.findings] == [("run", "error")]


def test_leds_and_logger(tmp_path):
    py = ('import time\nfrom arduino.app_utils import App, Leds, Logger\n\nlog = Logger("demo")\n\n'
          'def loop():\n    Leds.set_led1_color(1, 0, 0)\n    log.info("on")\n    time.sleep(1)\n\n'
          'App.run(user_loop=loop)\n')
    (tmp_path / "course.yml").write_text("- name: T\n  groups:\n    - arduino\n")
    (tmp_path / "01.md").write_text(f"```python\n{py}```\n")
    assert ccc.check([tmp_path], mp=None).findings == []
    (tmp_path / "01.md").write_text(f"```python\n{py.replace('set_led1_color', 'set_led3_color')}```\n")
    assert [f.rule for f in ccc.check([tmp_path], mp=None).findings] == ["run"]


def test_bridge_call_inside_a_python_handler(tmp_path):
    # Arduino: a provided Python function may notify, but must not call
    py = ('from arduino.app_utils import *\n\n'
          'def on_reading(value):\n    Bridge.call("set_led_state", value > 50)\n\n'
          'Bridge.provide("on_reading", on_reading)\nApp.run()\n')
    ino = GOOD_INO.replace("void loop() {\n}", 'void loop() {\n    Bridge.notify("on_reading", analogRead(A0));\n    delay(1000);\n}')
    report = lesson(tmp_path, py, ino)
    assert [(f.rule, f.level) for f in report.findings] == [("run", "error")]
    assert "nested calls" in report.findings[0].message
    assert lesson(tmp_path, py.replace("Bridge.call(", "Bridge.notify("), ino).findings == []


def test_code_board_lints_sketch_only_lessons(tmp_path):
    (tmp_path / "course.yml").write_text("- name: T\n  code_board: uno-q\n  groups:\n    - arduino\n")
    (tmp_path / "01.md").write_text("```cpp\n#include <Servo.h>\nvoid setup() {}\nvoid loop() {}\n```\n")
    assert [f.rule for f in ccc.check([tmp_path], mp=None).findings] == ["uno-q-servo"]
