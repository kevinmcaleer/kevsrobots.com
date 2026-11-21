---
title: Summary and Next Steps
description: A recap of the ClubKit Snakie plugin, the complete finished code to download, and ideas for plugins of your own
layout: lesson
type: page
cover: /learn/snakie_plugins/assets/banners/12_summary.jpg
date_updated: 2026-11-21
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

## You've extended your editor

You started with an empty folder. You now have a plugin that adds two commands, a linter with two rules and a quick fix, and a drawer of blocks. More to the point, you know how every part of it works, so you can build the plugin *your* club, class or workbench needs.

---

## What you learned

| Lesson | Skill |
|---|---|
| 1 to 2 | How the plugin host works, where plugins live, and the Plugin Rack |
| 3 | A plugin with one command, and the edit, save, Reload loop |
| 4 | Load errors, crashes and how to track down a bug |
| 5 | Reading code with patterns, messages and the status bar |
| 6 | Editing the open file safely |
| 7 | Linters, diagnostics, severities and the Problems panel |
| 8 | Quick fixes |
| 9 | Rules about your own hardware, and avoiding false alarms |
| 10 | Custom blocks |
| 11 | Sharing a plugin, and trust |
{:class="table table-single"}

---

## The whole plugin

Every plugin is built from the same four pieces.

| Decorator | The function receives | It returns |
|---|---|---|
| `@plugin.command(id, title)` | `ctx` | `message`, `edit`, `status`, or a list of them |
| `@plugin.linter(name)` | `ctx` | a list of `diagnostic`, each with optional `fix`es |
| `@plugin.blocks(name)` | nothing | a list of `block` |
{:class="table table-single"}

Here's the finished ClubKit. It's also in the download below.

```python
"""ClubKit: a Snakie plugin for our robot club."""

import re

from snakie import Context, block, diagnostic, edit, fix, message, plugin, status

HEADER = "# Robot Club\n"
PIN_USE = re.compile(r"Pin\(\s*(\d+)")
LONG_SLEEP = re.compile(r"\bsleep\(\s*(\d{3,})\s*\)")
OFF_HEADER = {23: "the power supply mode", 24: "USB power sensing", 29: "the VSYS voltage reading"}


# ---------- Commands ----------

@plugin.command("clubkit.header", "Add club header")
def add_header(ctx: Context):
    """Put the club's header comment at the top of the file."""
    if ctx.file.content.startswith(HEADER):
        return message("info", "This file already has the club header.")
    return [
        edit(HEADER + f"# File: {ctx.file.name}\n\n" + ctx.file.content),
        message("info", f"Added the club header to {ctx.file.name}."),
    ]


@plugin.command("clubkit.pins", "List the pins")
def list_pins(ctx: Context):
    """Report every GPIO pin the program uses."""
    pins = sorted({int(n) for n in PIN_USE.findall(ctx.file.content)})
    if not pins:
        return message("warning", "No pins found. Is this the right file?")
    names = ", ".join(f"GP{n}" for n in pins)
    return [
        message("info", f"{ctx.file.name} uses {len(pins)} pin(s): {names}"),
        status(f"Pins: {names}", tooltip="From ClubKit"),
    ]


# ---------- Linter ----------

@plugin.linter("club-rules")
def club_rules(ctx: Context):
    """Squiggle the mistakes our club makes most often."""
    found = []
    for number, line in enumerate(ctx.file.content.splitlines(), start=1):
        code = line.split("#")[0]

        match = LONG_SLEEP.search(code)
        if match:
            found.append(diagnostic(
                number,
                f"sleep({match.group(1)}) waits {match.group(1)} seconds. Did you mean milliseconds?",
                severity="warning",
                column=match.start() + 1,
                end_column=match.end() + 1,
                source="clubkit",
                fixes=[fix(f"Change to sleep_ms({match.group(1)})", f"sleep_ms({match.group(1)})")],
            ))

        for match in PIN_USE.finditer(code):
            pin = int(match.group(1))
            if pin in OFF_HEADER:
                found.append(diagnostic(
                    number,
                    f"GP{pin} is used by the Pico for {OFF_HEADER[pin]}. Pick another pin.",
                    severity="error",
                    column=match.start(1) + 1,
                    end_column=match.end(1) + 1,
                    source="clubkit",
                ))
    return found


# ---------- Blocks ----------

@plugin.blocks("ClubKit")
def club_blocks():
    """Blocks for the club's robot."""
    imports = [{"module": "clubbot"}]
    return [
        block("forward", "drive forward %1 cm", "clubbot.forward({CM})\n",
              args=[{"name": "CM", "kind": "number", "default": 20}],
              imports=imports, tooltip="Drive forward in a straight line."),
        block("turn", "turn %1 by %2 degrees", "clubbot.turn({DIR} * {DEG})\n",
              args=[{"name": "DIR", "kind": "choice", "options": [["right", "1"], ["left", "-1"]]},
                    {"name": "DEG", "kind": "number", "default": 90}],
              imports=imports, tooltip="Turn on the spot."),
        block("stop", "stop the motors", "clubbot.stop()\n",
              imports=imports, tooltip="Stop both motors."),
        block("distance", "distance ahead in cm", "clubbot.distance()",
              shape="value", output="Number",
              imports=imports, tooltip="How far away the nearest thing is."),
    ]
```

**Downloads**

- [clubkit.zip](/learn/snakie_plugins/assets/clubkit.zip), the finished plugin with a README. Unzip it into `.snakie/plugins`
- [patrol.py](/learn/snakie_plugins/assets/patrol.py), the sample robot program with its two mistakes

---

## Plugin ideas

ClubKit was a made-up example. Here are some real ones worth building.

1. **A project template.** A command that fills an empty file with your standard starting code: imports, pin constants and a `main()` loop
2. **A wiring checker.** A linter that knows the pin map of your robot and warns when a program uses a pin for the wrong job
3. **Name your numbers.** An `info` diagnostic on any bare `Pin(15)`, suggesting a named constant such as `BUZZER_PIN = 15`
4. **A loop guard.** A warning on `while True:` loops with no `sleep` inside, which can make a board hard to interrupt
5. **Kit blocks.** A drawer of blocks for a robot kit, a sensor board or a classroom library
6. **A marking helper.** For teachers: a command that reports whether a program has a header, comments and no warnings

Start with whichever mistake you've explained to someone three times this month.

---

## What's Next

- [Make Your Own Parts and Boards in Snakie](/learn/snakie_part_editor/) shows the other way to add blocks: a part that brings its own, which also works in the browser
- [Coding Faster in Snakie](/learn/snakie_code_tools/) covers the built-in linter, refactors and Git that your plugin sits alongside
- [Build and Animate a Robot in Snakie](/learn/snakie_build_urdf/) is the place to go if ClubKit has made you want a club robot
- The [Snakie Power User](/learn/learning_pathways/snakie-power-user.html) pathway collects the intermediate Snakie courses

Build a plugin, share it with someone, and tell us what it does. We'd love to see it.

---

## Check your understanding

{% quiz %}
? Which decorator would you use for a rule that underlines mistakes as you type?
- [ ] @plugin.command
- [x] @plugin.linter
- [ ] @plugin.blocks
> A linter runs by itself when the code changes and returns diagnostics.

? What do all plugin functions have in common?
- [ ] They change the editor directly
- [ ] They run on the board
- [x] They return a description of what they want, and Snakie does the work
> Commands return actions, linters return diagnostics and block providers return block descriptions.

? What is the best first plugin to write for your own group?
- [ ] The biggest one you can think of
- [x] One that catches a mistake you keep having to explain
- [ ] One that needs the network
> Small, specific and useful beats ambitious. Start with a real, repeated problem.
{% endquiz %}
