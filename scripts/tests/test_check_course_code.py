"""Tests for scripts/check_course_code.py. Run: python3 -m pytest scripts/tests"""
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import check_course_code as ccc  # noqa: E402

FIXTURE = HERE / "fixtures" / "code_check_course"


def rules(report):
    return sorted((f.line, f.level, f.rule) for f in report.findings)


def test_static_checks_without_micropython():
    report = ccc.check([FIXTURE], mp=None)
    found = rules(report)
    assert (18, "error", "micropython-api") in found        # time.strftime
    assert (22, "error", "micropython-import") in found     # datetime
    assert (26, "error", "micropython-api") in found        # print(flush=)
    assert report.blocks == 7                               # the skip block is left out


def test_localtime_attribute_is_flagged(tmp_path):
    (tmp_path / "course.yml").write_text("- name: T\n  groups:\n    - pico\n")
    (tmp_path / "01.md").write_text(
        "```python\nimport time\nt = time.localtime()\nprint(t.tm_hour)\n```\n")
    report = ccc.check([tmp_path], mp=None)
    assert [(f.line, f.rule) for f in report.findings] == [(4, "micropython-localtime")]


def test_cpython_course_is_not_held_to_micropython(tmp_path):
    (tmp_path / "course.yml").write_text("- name: T\n  groups:\n    - python\n")
    (tmp_path / "01.md").write_text("```python\nimport datetime\nprint(datetime.date.today())\n```\n")
    assert ccc.check([tmp_path], mp=None).findings == []


def test_code_target_front_matter(tmp_path):
    (tmp_path / "course.yml").write_text("- name: T\n  groups:\n    - micropython\n")
    (tmp_path / "01.md").write_text(
        "---\ntitle: x\ncode_target: cpython\n---\n```python\nimport datetime\n```\n")
    assert ccc.check([tmp_path], mp=None).findings == []


def test_syntax_error_and_placeholder(tmp_path):
    (tmp_path / "01.md").write_text(
        "```python\nwhile True\n    pass\n```\n\n```python\nspi = SPI(0, baudrate=1, ...)\n```\n")
    found = rules(ccc.check([tmp_path], mp=None))
    assert found == [(2, "error", "syntax"), (7, "warning", "syntax")]


MP = ccc.find_micropython(None)


@pytest.mark.skipif(not MP, reason="no MicroPython binary (scripts/get_micropython.sh)")
def test_runs_on_micropython():
    report = ccc.check([FIXTURE], mp=MP)
    found = rules(report)
    assert (7, "error", "run") in found    # ADC.read() isn't on the rp2 port
    assert (13, "error", "run") in found   # nor PWM.duty()
    # The no-snakie fragment and the endless blink loop are fine.
    assert not [f for f in report.findings if f.line > 30]
