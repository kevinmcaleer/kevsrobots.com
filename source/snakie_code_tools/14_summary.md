---
title: Summary and What's Next
description: Review the Snakie code tools you have learned, a workflow that uses them together, and where to go next
layout: lesson
type: page
cover: /learn/snakie_code_tools/assets/banners/14_summary.jpg
date_updated: 2026-11-21
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

You started this course with an editor. You're finishing it with a workshop.

---

## What you learned

| Tool | What it's for | Where |
|---|---|---|
| **Autocomplete** and signature help | Writing without looking things up | As you type, <kbd>Ctrl</kbd>+<kbd>Space</kbd> |
| **Hover** and **Help for symbol** | Reading documentation in place | Mouse, right-click |
| **Go to Definition** | Following code to where it's defined | <kbd>F12</kbd> |
| **Find and Replace** | Changing many places at once | <kbd>Ctrl</kbd>+<kbd>F</kbd> |
| **Problems** panel | Seeing every finding in one list | Bottom panel |
| **Refactoring hints** and **Tidy** | Smaller, faster MicroPython | Lightbulb, right-click |
| **YAML and JSON validation** | Settings files that load | Automatic |
| **Packages** | Installing libraries with mip | Activity bar |
| **Modules** | Seeing what's already on the board | Packages, Modules tab |
| **Compile to .mpy** | Less memory, faster start | Right-click a file |
| **Source Control** | History, diffs and undo | Activity bar, desktop |
| **Chat** | Explanations and first drafts | Editor top right, desktop |
{:class="table table-single"}

## A workflow

The tools are most useful in an order:

1. **Commit** what works
2. **Write** the next piece, with autocomplete and hover
3. **Run** it on the simulator or the board
4. **Check** the Problems panel. Fix errors, then consider the hints
5. **Read the diff**. Is it only what you meant to change?
6. **Commit** with a message that says why
7. When a module is finished, **compile** it

Steps 1 and 6 are the ones people skip, and the ones they wish they hadn't.

## Three ideas to keep

**Let the editor remember things.** You don't need to memorise every method of every class. You need to know how to ask.

**A hint is a lesson.** Every refactoring hint has a **Why?** article. Reading them is a short course in how MicroPython really works.

**Make changes you can undo.** Preview before applying. Commit before experimenting. Then nothing you try can cost you more than a few minutes.

## What's next

- [Snakie Instruments - Oscilloscope, Multimeter and Plotter](/learn/snakie_instruments/): measure what your tidy code is actually doing
- [Wire It Up - Electronics in Snakie](/learn/snakie_electronics/): build the robot this course's driver belongs to
- [Raspberry Pi Pico with MicroPython - GPIO Mastery](/learn/micropython_gpio/): more of the hardware side, including timers and interrupts
- [Learn MicroPython - The basics](/learn/micropython/): if any of the Python here felt new

## Final check

{% quiz %}
? Your robot worked yesterday and doesn't today. Which tool do you open first?
- [ ] Packages
- [x] Source Control, to read the diff
- [ ] The Sprite Editor
> The bug is somewhere in what changed, and the diff shows exactly that.

? A program fails with MemoryError while importing a large library. What might help?
- [ ] Adding more print statements
- [ ] Renaming the file
- [x] Compiling the library to .mpy
> A compiled module skips the memory-hungry compile step on the board.

? Before installing a library, where should you look?
- [x] The Modules tab, in case it's already in firmware or on the device
- [ ] The console
- [ ] The Help menu
> Many useful modules are built in, and the list depends on your board.
{% endquiz %}
