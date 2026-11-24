---
layout: lesson
title: Compiling to mpy
author: Kevin McAleer
type: page
cover: /learn/snakie_code_tools/assets/banners/09_compiling_to_mpy.jpg
date: 2026-11-21
previous: 08_the_modules_shelf.html
next: 10_version_control_with_git.html
description: Compile a MicroPython module to a compact .mpy file in Snakie to save
  memory and boot faster, and learn when you should and should not
percent: 60
duration: 5
quiz_total: 42
date_updated: 2026-11-24
navigation:
- name: Coding Faster in Snakie
- content:
  - section: Overview
    content:
    - name: Coding Faster in Snakie
      link: 00_intro.html
  - section: Writing
    content:
    - name: Smart Autocomplete
      link: 01_smart_autocomplete.html
    - name: Hover Help and Definitions
      link: 02_hover_help_and_definitions.html
    - name: Find and Replace
      link: 03_find_and_replace.html
  - section: Checking
    content:
    - name: Linting and the Problems Panel
      link: 04_linting_and_problems.html
    - name: MicroPython Refactors
      link: 05_micropython_refactors.html
    - name: YAML and JSON Validation
      link: 06_yaml_and_json.html
  - section: Libraries
    content:
    - name: Installing Packages
      link: 07_installing_packages.html
    - name: The Modules Shelf
      link: 08_the_modules_shelf.html
    - name: Compiling to mpy
      link: 09_compiling_to_mpy.html
  - section: Working Like a Pro
    content:
    - name: Version Control with Git
      link: 10_version_control_with_git.html
    - name: AI Chat and Suggestions
      link: 11_ai_chat_and_suggestions.html
    - name: Menus and Keyboard Shortcuts
      link: 12_menus_and_shortcuts.html
  - section: Project
    content:
    - name: Project - A Tidy Sensor Driver
      link: 13_project_sensor_driver.html
    - name: Summary and What's Next
      link: 14_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to compile a .py file to .mpy and explain when it is worth doing.

Every time your board starts, MicroPython reads your `.py` files and compiles them to **bytecode** before it can run them. For a large library that takes time and, more importantly, memory. You can do that step once, on your computer, and give the board the result.

---

## What an .mpy file is

A `.mpy` file is a module that's already been compiled. The board loads it directly: no reading, no parsing, no compiling.

| | `.py` | `.mpy` |
|---|---|---|
| Readable and editable | Yes | No |
| Compiled on the board at every start | Yes | No, already done |
| Memory needed to import | More | Less |
| Size on flash | Larger | Smaller |
| Works on any MicroPython version | Yes | Only matching versions |
{:class="table table-single"}

The memory row is the one that matters most. Compiling needs working space. On a big module, that temporary peak can be the largest thing your program ever asks for, and it's a classic cause of a confusing `MemoryError` during `import`, before any of your own code has run.

## Compiling in Snakie

In the **Local Files** panel, right-click a `.py` file.

![The Local Files right-click menu with Compile to .mpy](/learn/snakie_code_tools/assets/tree_menu.jpg){:class="img-fluid w-100"}

Choose **Compile to .mpy**. A new file appears beside the original with the same name and the new ending. Our 726-byte `hcsr04.py` becomes a 359-byte `hcsr04.mpy`. That's half the size.

Your program doesn't change at all:

```python
from hcsr04 import HCSR04
```

MicroPython looks for `hcsr04.py` first and `hcsr04.mpy` second. So upload the `.mpy` **instead of** the `.py`, not alongside it. If both are on the board, the `.py` wins and you've gained nothing.

## Looking inside

A `.mpy` is not text, but Snakie can still tell you about it. Double-click the new file.

![Snakie's read-only view of a compiled module](/learn/snakie_code_tools/assets/mpy_view.jpg){:class="img-fluid w-100"}

The top says it plainly: *Compiled bytecode, shown read-only. The original source is not inside a .mpy — names and constants survive compilation, the code you wrote does not.*

| Field | Our file | Meaning |
|---|---|---|
| Compiled for | MicroPython | Not CircuitPython, which marks its files differently |
| Format | .mpy version 6 | The bytecode version. See below |
| Code | portable bytecode | Runs on any chip |
| Compiled from | hcsr04.py | The source file's name |
| Size | 359 bytes | |
{:class="table table-single"}

Under **DEFINES** is everything the module offers: the `HCSR04` class, with `__init__(self, trigger, echo, timeout_us=…)`, `_pulse(self)` and `distance_mm(self)`. The names are real, read from the bytecode. Default values aren't stored, which is why `timeout_us` shows `…`.

This view is handy for a `.mpy` someone else gave you. You can see what it contains and which version it was built for before you put it on a board.

## The version catch

A `.mpy` is tied to a **bytecode version**. Version 6 is used by current MicroPython releases. A file built for one version won't load on firmware that expects another, and you'll see:

```
ValueError: incompatible .mpy file
```

So when you update a board's firmware to a release that changes the format, recompile. Keeping the `.py` makes that a ten-second job.

## When to compile

Do compile:

- Libraries and drivers that are finished and stable
- Big modules that cause `MemoryError` on import
- Code you're handing to others to use, not to read

Don't compile:

- `main.py`. MicroPython looks for it by that exact name
- Anything you're still changing. You lose the ability to edit on the board, and tracebacks are less helpful
- Code you want learners to read

---

## Try it Yourself

1. Compile a module of your own. What's the size before and after?
2. Open the `.mpy` and compare the **DEFINES** list with the source. Is anything missing?
3. Put both `hcsr04.py` and `hcsr04.mpy` on a board, change a `print` in the `.py`, and import it. Which one ran?

## Common Issues

- **Problem**: The menu says **Compile to .mpy (unavailable)**.
- **Solution**: Open your project as a folder from the Files panel.
- **Why**: The compiler needs a real folder to write the new file into.

- **Problem**: `ValueError: incompatible .mpy file` on the board.
- **Solution**: Update the board's firmware, or recompile after updating Snakie.
- **Why**: The file's bytecode version doesn't match what the firmware expects.

- **Problem**: My changes aren't taking effect.
- **Solution**: Recompile, and upload the new `.mpy`. Check there's no stale copy on the board.
- **Why**: The `.mpy` is a snapshot of the source at the moment you compiled it.

## Check your understanding

{% quiz %}
? What is the main benefit of a .mpy file on a small board?
- [ ] It's easier to read
- [x] It imports without compiling on the board, saving memory and time
- [ ] It runs on CircuitPython too
> The compile step, and the memory it needs, has already happened on your computer.

? Both hcsr04.py and hcsr04.mpy are on the board. Which is imported?
- [x] hcsr04.py
- [ ] hcsr04.mpy
- [ ] Neither
> MicroPython looks for the .py first, so upload the .mpy instead of it.

? Which file should you not compile?
- [ ] A finished driver
- [ ] A large stable library
- [x] main.py
> The board looks for main.py by name, and it's the file you change most.
{% endquiz %}
