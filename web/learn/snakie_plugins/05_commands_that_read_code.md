---
layout: lesson
title: Commands That Read Code
author: Kevin McAleer
type: page
cover: /learn/snakie_plugins/assets/banners/05_commands_that_read_code.jpg
date: 2026-11-21
previous: 04_when_plugins_break.html
next: 06_commands_that_change_code.html
description: Write a Snakie plugin command that searches a MicroPython program for
  the pins it uses and reports them in a message and the status bar
percent: 42
duration: 5
quiz_total: 36
date_updated: 2026-11-24
navigation:
- name: Extend Snakie with Python Plugins
- content:
  - section: Overview
    content:
    - name: Extend Snakie with Python Plugins
      link: 00_intro.html
    - name: How Plugins Work
      link: 01_how_plugins_work.html
    - name: The Plugin Rack
      link: 02_the_plugin_rack.html
  - section: Commands
    content:
    - name: Your First Plugin
      link: 03_your_first_plugin.html
    - name: When Plugins Break
      link: 04_when_plugins_break.html
    - name: Commands That Read Code
      link: 05_commands_that_read_code.html
    - name: Commands That Change Code
      link: 06_commands_that_change_code.html
  - section: Linters
    content:
    - name: Your First Linter
      link: 07_your_first_linter.html
    - name: Quick Fixes
      link: 08_quick_fixes.html
    - name: Rules for Your Hardware
      link: 09_rules_for_your_hardware.html
  - section: Blocks and Sharing
    content:
    - name: Blocks from a Plugin
      link: 10_blocks_from_a_plugin.html
    - name: Sharing Your Plugin
      link: 11_sharing_your_plugin.html
    - name: Summary and Next Steps
      link: 12_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to write a plugin command that searches a program for the pins it uses and reports them in a message and the status bar.

## A useful question

"Which pins does this program use?" Every club leader asks it, usually while holding a robot with a wire in the wrong hole.

Let's make ClubKit answer it. The command will read the program, find every pin and report back.

---

## Finding pins in text

`ctx.file.content` is one long string. In MicroPython a pin nearly always looks like `Pin(15`, so we want every number that follows `Pin(`.

Python's `re` module finds patterns like that. Try this in a Python prompt on your computer:

```python
import re

code = "led = Pin(25, Pin.OUT)\nbuzzer = Pin(15, Pin.OUT)"
print(re.findall(r"Pin\(\s*(\d+)", code))
```

It prints `['25', '15']`. Here's how to read the pattern:

| Piece | Means |
|---|---|
| `Pin\(` | the letters `Pin` then an opening bracket (the `\` says "a real bracket") |
| `\s*` | any spaces, or none |
| `(\d+)` | one or more digits, and the brackets say "this is the bit I want" |
{:class="table table-single"}

`Pin.OUT` isn't matched, because there's no bracket straight after `Pin`.

---

## The command

Replace the contents of `__init__.py` with this. **Say hi** has done its job, so it goes.

```python
"""ClubKit: a Snakie plugin for our robot club."""

import re

from snakie import Context, message, plugin

PIN_USE = re.compile(r"Pin\(\s*(\d+)")


@plugin.command("clubkit.pins", "List the pins")
def list_pins(ctx: Context):
    """Report every GPIO pin the program uses."""
    pins = sorted({int(n) for n in PIN_USE.findall(ctx.file.content)})
    if not pins:
        return message("warning", "No pins found. Is this the right file?")
    names = ", ".join(f"GP{n}" for n in pins)
    return message("info", f"{ctx.file.name} uses {len(pins)} pin(s): {names}")
```

The busy line is the one that builds `pins`. Reading it from the inside out:

1. `PIN_USE.findall(...)` gives the numbers as text: `['25', '15', '29']`
2. `int(n) for n in ...` turns each into a number
3. The curly brackets make a **set**, which throws away repeats. A pin used twice is listed once
4. `sorted(...)` puts them in order

Then there are two ways out. No pins at all earns a **warning**, because the user has probably run the command on the wrong file. Otherwise the pins are joined into a sentence.

Save, Reload, open `patrol.py`, and run **List the pins**.

---

## Two answers at once

A command can return a **list** of things, and Snakie handles each in turn. Let's also put the pins in the status bar, where they stay in view while you wire up.

Add `status` to the import:

```python
from snakie import Context, message, plugin, status
```

And change the last line of the function:

```python
    return [
        message("info", f"{ctx.file.name} uses {len(pins)} pin(s): {names}"),
        status(f"Pins: {names}", tooltip="From ClubKit"),
    ]
```

Save, Reload and run it again.

![The pin list in the panel and the status bar](/learn/snakie_plugins/assets/pins.jpg){:class="img-fluid w-100"}

The message is at the bottom of the panel, and `Pins: GP15, GP25, GP29` is in the status bar at the bottom left of the window.

`status` takes a few optional extras:

| Extra | Does |
|---|---|
| `tooltip="..."` | Text shown when you hover over it |
| `href="https://..."` | Makes it a link |
| `priority=1` | Higher numbers win when plugins compete for the space |
{:class="table table-single"}

---

## Message or status?

- Use a **message** for an answer to something the user just asked
- Use a **status** for a fact worth keeping in view
- Use a **warning** or **error** message when the user needs to do something

---

## Try it Yourself

1. Add a second `Pin(15, ...)` line to `patrol.py` and run the command. Is GP15 listed once or twice?
2. Run the command on a file with no pins
3. Write a new command, `clubkit.count`, titled "Count the lines", that reports how many lines the file has and how many are blank. `ctx.file.content.splitlines()` gives you a list of lines

---

## Common Issues

**Problem**: `Pin(LED_PIN)` isn't found.
**Solution**: That's expected with this pattern. It only finds pins written as numbers.
**Why**: The plugin reads text. It doesn't run the program, so it can't know what `LED_PIN` holds.

**Problem**: `NameError: name 're' is not defined`.
**Solution**: Add `import re` at the top of the file.
**Why**: A plugin is a normal Python file and needs its own imports.

**Problem**: The status bar text has gone.
**Solution**: Run the command again.
**Why**: Another plugin's status can replace it, and it isn't saved between sessions.

---

## Check your understanding

{% quiz %}
? How does a command send back a message and a status together?
- [ ] It calls return twice
- [x] It returns a list containing both
- [ ] It can't
> Return a list and Snakie handles each item in turn.

? Why does the command use a set when collecting pins?
- [ ] Sets are faster to print
- [x] A set drops repeats, so each pin is listed once
- [ ] Snakie only accepts sets
> A pin that appears on several lines should still be reported once.

? The command can't find the pin in Pin(LED_PIN). Why?
- [x] The plugin reads the text and doesn't run the program
- [ ] The regular expression has a bug
- [ ] Variables aren't allowed in MicroPython
> A plugin sees source code as text. It doesn't know the value of a variable.
{% endquiz %}
