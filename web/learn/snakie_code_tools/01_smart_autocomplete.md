---
layout: lesson
title: Smart Autocomplete
author: Kevin McAleer
type: page
cover: /learn/snakie_code_tools/assets/banners/01_smart_autocomplete.jpg
date: 2026-11-21
previous: 00_intro.html
next: 02_hover_help_and_definitions.html
description: Use Snakie's MicroPython-aware autocomplete and signature help to write
  hardware code faster and with fewer typos
percent: 12
duration: 4
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

Autocomplete is the editor finishing your word. In Snakie it does a bit more than that, because it has read the MicroPython documentation so you don't have to.

---

## Completing a method

Open a new file and type these two lines, then start a third with `left.`

<!-- code-check: skip -->
```python
from machine import Pin, PWM
left = PWM(Pin(10))
left.
```

The moment you type the dot, a list appears.

![Autocomplete listing the methods of a PWM object](/learn/snakie_code_tools/assets/autocomplete.jpg){:class="img-fluid w-100"}

Those are the methods of a `PWM` object: `deinit`, `duty_ns`, `duty_u16`, `freq`, `init`. Snakie worked out that `left` is a PWM because of the line above.

| Key | What it does |
|---|---|
| <kbd>↑</kbd> <kbd>↓</kbd> | Move through the list |
| <kbd>Tab</kbd> or <kbd>Enter</kbd> | Accept the highlighted item |
| <kbd>Esc</kbd> | Close the list |
| <kbd>Ctrl</kbd>+<kbd>Space</kbd> | Bring the list back |
{:class="table table-single"}

Keep typing to narrow it down. `left.du` leaves only the duty methods.

Beside the highlighted item is a short description that starts with **MicroPython ·**. That label matters. It means the suggestion comes from MicroPython's own definitions, not from desktop Python, which has a different set of modules.

## Completing an import

Autocomplete also knows what lives in each module. Type `from machine import ` and press <kbd>Ctrl</kbd>+<kbd>Space</kbd>.

![Autocomplete listing the classes in the machine module](/learn/snakie_code_tools/assets/autocomplete_import.jpg){:class="img-fluid w-100"}

`ADC`, `I2C`, `Pin`, `PWM`, `RTC`, `SPI`, `Timer` and the rest. It's the fastest way to answer "what was that class called again?"

The same works after a module name. `time.` lists `sleep`, `sleep_ms`, `sleep_us`, `ticks_ms`, `ticks_diff` and friends.

## Signature help

Pick `freq` and type the opening bracket.

![Signature help for PWM.freq](/learn/snakie_code_tools/assets/signature.jpg){:class="img-fluid w-100"}

A panel shows the function's **signature**, `PWM.freq(value: int)`, and what it does: *Get or set the current frequency of the PWM output.* You now know it wants one whole number, without opening a browser.

When a function can be called in more than one way, the panel shows a counter such as **2/2** and you can step through the versions with the arrow keys.

## Try it for real

Here's a short program. Don't paste it. Type it into the full app and let autocomplete do as much of the work as it can.

{% capture snakie_code %}
from machine import Pin, PWM
import time

led = PWM(Pin(15))
led.freq(1000)

for level in range(0, 101, 20):
    led.duty_u16(level * 65535 // 100)
    print("Brightness", level, "%")
    time.sleep_ms(200)

led.duty_u16(0)
{% endcapture %}
{% include snakie.html code=snakie_code name="fade.py" view="code" %}

Count how many characters you actually typed for `led.duty_u16`. Four or five is normal.

## What it can't do

Autocomplete follows simple assignments like `left = PWM(Pin(10))`. It won't always follow a value through a function call or a list. If the list doesn't appear after a dot, the editor has lost track of what the variable is. Your code may still be fine.

---

## Try it Yourself

1. Type `import ` and press <kbd>Ctrl</kbd>+<kbd>Space</kbd>. Find three modules you've never used.
2. Type `time.ti` and read the description of `ticks_diff`. What is it for?
3. Type `Pin(` and read the signature help. Which argument sets a pull-up?

## Common Issues

**Problem**: The list doesn't appear.
**Solution**: Press <kbd>Ctrl</kbd>+<kbd>Space</kbd>. Check the file name ends in `.py`.
**Why**: Completions are for Python files, and the list closes when you press <kbd>Esc</kbd> or click away.

**Problem**: I accepted the wrong suggestion.
**Solution**: Press <kbd>Ctrl</kbd>+<kbd>Z</kbd> and carry on typing to narrow the list first.
**Why**: <kbd>Enter</kbd> accepts whatever is highlighted, which is the top item until you move.

**Problem**: A suggestion is labelled CircuitPython.
**Solution**: Ignore it if your board runs MicroPython.
**Why**: Snakie supports both, and their module names differ. The label tells you which family a suggestion belongs to.

## Check your understanding

{% quiz %}
? Which key brings the suggestion list back after you've closed it?
- [ ] F1
- [x] Ctrl+Space
- [ ] Ctrl+S
> Ctrl+Space asks for suggestions wherever the cursor is.

? What does signature help show?
- [ ] Your program's output
- [ ] The pins on your board
- [x] The arguments a function expects and what it does
> It appears when you type the opening bracket of a call.

? Why did Snakie know that left has a duty_u16 method?
- [x] It saw left = PWM(...) on the line above
- [ ] It asked the board
- [ ] Every variable has one
> The editor follows simple assignments to work out what a name refers to.
{% endquiz %}
