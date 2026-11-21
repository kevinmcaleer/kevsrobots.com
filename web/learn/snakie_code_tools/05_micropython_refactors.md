---
layout: lesson
title: MicroPython Refactors
author: Kevin McAleer
type: page
cover: /learn/snakie_code_tools/assets/banners/05_micropython_refactors.jpg
date: 2026-11-21
previous: 04_linting_and_problems.html
next: 06_yaml_and_json.html
description: Use Snakie's MicroPython refactoring hints, quick fixes with a preview,
  and Tidy this file to make code smaller and faster without changing what it does
percent: 36
duration: 6
quiz_total: 42
date_updated: 2026-11-21
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

**Refactoring** means changing how code is written without changing what it does. Snakie has a set of refactorings written specifically for microcontrollers. They know things a general Python tool has no reason to care about, like how much memory a float costs on a Pico.

---

## Hints

Open the program below in the full app and click the **Problems** tab. It runs perfectly well. Snakie still has four things to say about it.

{% capture snakie_code %}
from machine import Pin, PWM
import time

LEFT_MOTOR_PIN = 10
RIGHT_MOTOR_PIN = 12

left = PWM(Pin(LEFT_MOTOR_PIN))
right = PWM(Pin(RIGHT_MOTOR_PIN))


def drive(speed):
    duty = int(speed * 65535 / 100)
    left.duty_u16(duty)
    right.duty_u16(duty)
    print("Driving at", speed, "%")


for speed in (20, 40, 60, 0):
    drive(speed)
    time.sleep(0.1)
{% endcapture %}
{% include snakie.html code=snakie_code name="tidy_me.py" view="code" %}

The rows marked ✦ and **refactor** are hints. For this file:

| Line | Hint |
|---|---|
| 4, 5 | `LEFT_MOTOR_PIN` never changes. `const()` inlines it and saves the lookup |
| 12 | `int(a / b)` divides in floats and truncates. `a // b` stays in integer maths |
| 20 | `sleep(0.1)` is a float. `sleep_ms(100)` is the portable, allocation-free spelling |
{:class="table table-single"}

None of these is a bug. Each is a small cost, paid every time round a loop, on a chip with very little to spare.

## Why?

Most hint rows end with a **Why?** link. Click one.

![The help article behind a refactoring hint](/learn/snakie_code_tools/assets/why.jpg){:class="img-fluid w-100"}

The Help panel opens with an article on that exact refactoring: a before and after, why it matters on MicroPython, and when Snakie will and won't offer it. This is where the learning is. The `const()` article explains that on desktop Python a constant is just a global variable, but MicroPython's *compiler* understands `const()` and folds the value straight into the bytecode, so the name lookup disappears from every loop.

Read the article the first time you meet each hint. After that you'll recognise the pattern yourself.

## Applying a fix

Put the cursor on line 4. A lightbulb appears at the start of the line. Click it, or press <kbd>Ctrl</kbd>+<kbd>.</kbd>

![The lightbulb menu offering to wrap a constant in const](/learn/snakie_code_tools/assets/quickfix.jpg){:class="img-fluid w-100"}

Choose **Wrap the constant in const()**. Snakie doesn't change your file yet. It shows you what it's about to do.

![A preview of the change before it is applied](/learn/snakie_code_tools/assets/refactor_preview.jpg){:class="img-fluid w-100"}

Red lines go, green lines arrive. Here that's **+2 −1**: the assignment is rewritten, and an import is added because the file didn't have one.

```python
from micropython import const

LEFT_MOTOR_PIN = const(10)
```

**Apply** makes the change. **Cancel** leaves the file alone. Either way you saw it first, and <kbd>Ctrl</kbd>+<kbd>Z</kbd> undoes it afterwards.

## Tidy this file

Fixing hints one at a time teaches you the most. Once you know them, there's a faster way. Right-click anywhere in the editor and choose **Tidy this file (Snakie)**.

![Tidy this file previewing four safe refactorings](/learn/snakie_code_tools/assets/tidy.jpg){:class="img-fluid w-100"}

Snakie gathers the refactorings it considers **safe**, meaning it can prove they don't change behaviour, and previews them as one change. Press **Apply** and they're all done.

If some hints remain afterwards, run Tidy again. It applies changes that don't overlap, so a file with several constants can take more than one pass.

## The hints it won't fix

Look back at the robot's `main.py` from the last lesson. One hint said:

> This loop blocks for 100 ms every pass. Check `ticks_ms()` instead and the rest of the board keeps running

There's no lightbulb for that one. Turning a sleeping loop into a non-blocking one depends on what else your program needs to do, and only you know that. Snakie points and explains. You decide. We'll make exactly this change by hand in the project.

## Asking for more

The right-click menu also has **Refactor… (Snakie)**, <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>R</kbd>. It lists every refactoring available at the cursor, including ones that aren't shown as hints. If nothing applies there, it says **No refactorings available**.

Two settings control how much you see. Open **Settings**, then **Editor**, and find **Refactoring hints**:

- **Show style suggestions in Problems** adds general Python advice, such as guard clauses, simpler loops and names for magic numbers. The MicroPython hints are always shown
- **Always preview a refactoring before applying it** keeps the preview step. Leave it on while you're learning

---

## Try it Yourself

1. Apply the `const()` fix to one constant using the lightbulb, then use Tidy for the rest. Run the program to confirm it behaves the same.
2. Read the **Why?** article for the `sleep_ms` hint. What does "allocation-free" mean?
3. Change `int(speed * 65535 / 100)` to use `//`, then think: when would `int(a / b)` and `a // b` give different answers?

## Common Issues

**Problem**: There's no lightbulb on a hinted line.
**Solution**: Read the hint and its **Why?** article, and make the change by hand.
**Why**: Some hints need a decision only you can make, so Snakie explains instead of rewriting.

**Problem**: My program broke after `const()`.
**Solution**: Undo it, and check whether the name is changed anywhere else, including from another file.
**Why**: A `const` is fixed at compile time. Snakie only offers it when the file never reassigns the name, but it can't see other files that might.

**Problem**: Tidy changed more than I expected.
**Solution**: Press <kbd>Ctrl</kbd>+<kbd>Z</kbd>, then apply the fixes one at a time from the lightbulb.
**Why**: Tidy applies every safe refactoring at once. The preview lists them all, so read it before pressing Apply.

## Check your understanding

{% quiz %}
? What does refactoring change?
- [ ] What the program does
- [x] How the code is written, not what it does
- [ ] The board's firmware
> A good refactoring leaves behaviour exactly as it was.

? What happens when you choose a fix from the lightbulb?
- [ ] The file changes immediately
- [ ] The board restarts
- [x] A preview shows the change, and you choose Apply or Cancel
> You always see the red and green lines first.

? Why doesn't Snakie rewrite a blocking loop for you?
- [x] The right answer depends on what else your program must do
- [ ] It's impossible
- [ ] It would take too long
> Snakie only auto-applies changes it can prove are safe.
{% endquiz %}
