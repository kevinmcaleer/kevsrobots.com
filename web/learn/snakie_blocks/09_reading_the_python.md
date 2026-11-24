---
layout: lesson
title: Reading the Python
author: Kevin McAleer
type: page
cover: /learn/snakie_blocks/assets/banners/09_reading_the_python.jpg
date: 2026-11-20
previous: 08_my_parts.html
next: 10_simple_and_advanced_blocks.html
description: Use hover highlighting, click a line and Show me the Python in Snakie
  to connect every block to the code it wrote
percent: 70
duration: 6
quiz_total: 36
date_updated: 2026-11-24
navigation:
- name: Snakie Blocks - From Blocks to Python
- content:
  - section: Overview
    content:
    - name: Snakie Blocks - From Blocks to Python
      link: 00_intro.html
    - name: The Blocks Workspace
      link: 01_the_blocks_workspace.html
  - section: Blocks Basics
    content:
    - name: Turtle Blocks - Draw a Square
      link: 02_turtle_draw_a_square.html
    - name: Variables, Maths and Loops
      link: 03_variables_maths_and_loops.html
    - name: Making Decisions
      link: 04_making_decisions.html
  - section: Blocks and Hardware
    content:
    - name: Hardware Blocks - LEDs and Buttons
      link: 05_leds_and_buttons.html
    - name: PWM, Servos and Buzzers
      link: 06_pwm_servos_and_buzzers.html
    - name: Functions
      link: 07_functions.html
    - name: My Parts
      link: 08_my_parts.html
  - section: From Blocks to Python
    content:
    - name: Reading the Python
      link: 09_reading_the_python.html
    - name: Simple and Advanced Blocks
      link: 10_simple_and_advanced_blocks.html
  - section: Project
    content:
    - name: Project - Reaction Timer
      link: 11_project_reaction_timer.html
    - name: Summary and Next Steps
      link: 12_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to use hover, click-a-line and Show me the Python to match each block to the Python it writes, and edit in either pane.

You've been glancing at the Python all along. This lesson is about reading it on purpose, using three tools Snakie gives you.

---

## Tool 1: hover a block

Open any program in the split view and move your mouse over a block. The lines that block wrote **light up** in the Python.

Hover a **repeat** block and several lines light up: the `for` line and everything inside it. That's indentation made visible. The block's mouth and the indented lines are the same thing.

## Tool 2: click a line

It works the other way too. Click a line of Python and its block is **selected** on the canvas. The code pane even reminds you: *click a line to find its block*.

This is the one to use when the console shows an error with a line number. Click that line, and you're looking at the block that caused it.

## Tool 3: Show me the Python

Right-click any block and choose **Show me the Python**. Snakie shows just that block's lines, on their own. Handy in a long program, when the block you care about is one of forty.

## Practise on this

Open this program with **Open in Snakie**, switch to the split view, and try all three tools.

{% capture snakie_code %}
import time
from machine import Pin
from snakie import Led
pin_14 = Pin(14, Pin.IN, Pin.PULL_UP)
led_15 = Led(pin=Pin(15, Pin.OUT))
while True:
    if pin_14.value() == 0:
        led_15.set(True)
    else:
        led_15.set(False)
    time.sleep_ms(50)
{% endcapture %}
{% include snakie.html code=snakie_code name="button.py" view="split" %}

## The shape of a Snakie program

Every blocks program is laid out the same way, and now you can name each part:

| Part | Example | Where it comes from |
|---|---|---|
| Imports | `import time` | Added when a block needs them |
| Setup lines | `pin_14 = Pin(14, Pin.IN, Pin.PULL_UP)` | Added the first time a block uses a pin |
| Your program | `while True:` and everything after | Your blocks, top to bottom |
{:class="table table-single"}

Most MicroPython programs you'll find anywhere follow this layout: imports, then setup, then a loop.

## Type in the other pane

Both panes are editable. Click in the Python and change `50` to `500`. The **wait** block updates.

Now try something bigger. At the end of the program, on a new line indented to match the `time.sleep_ms` line, type:

```python
    print("checking")
```

A print block appears in the stack. You just wrote a block by typing.

There's no "convert to Python" button in Snakie, and that's deliberate. You don't have to decide you're ready. Change a number in whichever pane you're looking at. Type a line when no block does what you want. Drag the divider towards the code as you start to prefer it. One day you'll notice you haven't touched the blocks for a while.

## It's just a .py file

Save a blocks program and look at it in the **Code** workspace. It's an ordinary Python file. Right at the bottom there's a comment where Snakie remembers how your blocks were arranged. Python ignores comments, so the file:

- runs on a Pico if you copy it straight across
- survives email, USB sticks and copy and paste
- opens in any text editor

And if that comment ever goes missing, Snakie works the blocks out from the code. Nothing is lost.

## When Snakie isn't sure

Sometimes you'll type Python that could be shown as blocks in more than one way. Snakie still draws blocks for it, but marks them as its *reading* of your code and tells you that editing them won't change your Python. Your code is safe. Carry on in the code pane, or rewrite the line the way the block would have written it.

That's why this course writes a pressed button as `pin_14.value() == 0`: it matches a block exactly.

---

## Try it Yourself

1. Hover every block in the button program and say out loud which lines will light up *before* they do.
2. Click the `else:` line. Which block is selected?
3. Add a second `print` by typing it, then delete it by dragging its block to the bin.
4. Challenge: cover the Python pane with your hand, build three blocks, and write down the Python you expect. Then check.

## Common Issues

- **Problem**: I typed some Python and the blocks went grey.
- **Solution**: Check the Problems tab for a syntax error, and fix the line.
- **Why**: Half-typed Python can't be turned into blocks until it makes sense again.

- **Problem**: My typed line became a grey Python block.
- **Solution**: That's fine. It still runs.
- **Why**: There isn't a matching coloured block for every line of Python, so Snakie keeps it exactly as you wrote it.

- **Problem**: A banner says the file uses advanced blocks.
- **Solution**: Click to show them in the toolbox.
- **Why**: Your code used something from the advanced set. The next lesson covers those.

## Check your understanding

{% quiz %}
? What happens when you hover a block in the split view?
- [ ] The block is deleted
- [x] The lines it wrote light up in the Python
- [ ] The program runs
> Hovering links each block to its lines of code.

? The console reports an error on line 7. What's the quickest way to find the block?
- [ ] Count the blocks from the top
- [x] Click line 7 in the Python pane
- [ ] Start again
> Clicking a line selects the block that wrote it.

? What kind of file is a saved blocks program?
- [ ] A special Snakie project file
- [x] An ordinary .py file
- [ ] A picture
> The blocks live in a comment at the bottom of a normal Python file.
{% endquiz %}
