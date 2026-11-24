---
title: Simple and Advanced Blocks
description: Switch on the advanced blocks in Snakie for lists and more, and use the grey Python blocks when the block you need does not exist
layout: lesson
type: page
cover: /learn/snakie_blocks/assets/banners/10_simple_and_advanced_blocks.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to switch on advanced blocks, use lists, and put your own code in a grey Python block.

Snakie starts you with a small set of blocks so the toolbox isn't overwhelming. When you're ready, there's a lot more behind one switch.

---

## Two tiers

A fresh Snakie shows the **simple** blocks. Under the toolbox is a switch called **Show advanced blocks**. Turn it on and the drawers grow.

| Tier | What you get |
|---|---|
| Simple | Everything used so far in this course |
| Advanced | Classes, try, files, slices, bitwise maths, global, and the grey Python blocks |
{:class="table table-single"}

Two things stay the same whichever way the switch is set:

- A program that uses advanced blocks still **opens and runs** with the switch off. The switch changes what you can reach for, not what Snakie understands
- The Python pane always shows everything

## Lists

A list holds several values in order. A tune is a list of notes. A route is a list of moves.

![A list of notes as blocks](/learn/snakie_blocks/assets/blocks_lists.jpg){:class="img-fluid w-100"}

The **create list with** block has a gear for adding more slots. The **for each item in list** loop then runs once for every item.

{% capture snakie_code %}
from machine import PWM, Pin
from snakie import Buzzer
buzzer_16 = Buzzer(PWM(Pin(16)))
notes = [262, 294, 330, 349, 392]
for note in notes:
    buzzer_16.tone(note, 200)
    print("Playing", note)
{% endcapture %}
{% include snakie.html code=snakie_code name="scale.py" view="split" %}

- `notes = [262, 294, 330, 349, 392]` is the list. Square brackets, commas between the items
- `for note in notes:` takes each item in turn and calls it `note`

Compare that to five separate buzzer blocks. To change the tune, you change one line.

## When the block you need doesn't exist

At the bottom of the toolbox is the **Python** drawer. With advanced blocks off, it tells you how to switch them on:

![The Python drawer with advanced blocks off](/learn/snakie_blocks/assets/toolbox_python.jpg){:class="img-fluid w-100"}

With them on, you get the grey blocks:

![The grey Python blocks](/learn/snakie_blocks/assets/toolbox_python_adv.jpg){:class="img-fluid w-100"}

Grey blocks are grey because they *are* code. Whatever you type in one goes into the program exactly as written.

| Block | What it's for |
|---|---|
| your Python here | One line of Python, in the stack |
| a value in Python | A piece of Python that works out a value, for a socket |
| a Python block | A line that ends in a colon, with a mouth for the lines inside it |
| # a note | A comment |
| import | Bring in a module |
{:class="table table-single"}

Click into one and a small code editor opens, with autocomplete. Type `machine.` and the list of choices appears.

This means **nothing is impossible in blocks**. A sensor nobody has made blocks for, a line from a tutorial, a module you found online: all of it works.

## A sensor with a grey block

Here's a light sensor on GP26 sending its readings to Snakie's plotter. The coloured blocks do the loop and the wait. A grey block does the one line that didn't have a block to hand.

![A sensor program that mixes coloured and grey blocks](/learn/snakie_blocks/assets/blocks_sensor.jpg){:class="img-fluid w-100"}

{% capture snakie_code %}
import time
from instruments import plot
from machine import ADC, Pin
adc_26 = ADC(Pin(26))
while True:
    plot(light=adc_26.read_u16())
    time.sleep_ms(100)
{% endcapture %}
{% include snakie.html code=snakie_code name="sensor.py" view="split" toolbox="advanced" %}

- `ADC(Pin(26))` sets GP26 up to measure a voltage
- `adc_26.read_u16()` reads it, as a number from 0 to 65535
- `plot(light=...)` draws it on the plotter instrument, on a line labelled `light`

Run it in the box and the readings scroll past in the console as `SNK PLOT` lines. To see them as a graph, click **Open in Snakie**, run it there, and open the **Plotter** from the instrument dock.

## Rules for grey blocks

- **One line per block.** Paste several lines and you get one block for each
- Lines that need indenting go **inside** a Control block or a grey Python block with a mouth, just as Python would indent them
- Snakie checks what you type and puts a note on the block if something looks wrong, such as *This round bracket ( is never closed.* It's advice. The code is written either way

---

## Try it Yourself

1. Turn **Show advanced blocks** on and see which drawers have grown.
2. Add three more notes to the list to finish the scale: 440, 494, 523.
3. Drag out a **# a note** block and describe what your program does.
4. Challenge: use a **your Python here** block to add `print(len(notes))` and find out what `len` does.

## Common Issues

- **Problem**: The Python drawer has no blocks in it.
- **Solution**: Turn on **Show advanced blocks**.
- **Why**: The grey blocks are part of the advanced set.

- **Problem**: A grey block has a warning on it.
- **Solution**: Read the sentence. It usually names the missing bracket or quote.
- **Why**: Snakie checks the line as you type.

- **Problem**: `IndentationError` after adding a grey block.
- **Solution**: Put one line in each block, and put lines that belong to a loop inside the loop block.
- **Why**: The blocks do the indenting. Typed spaces at the start of a line confuse it.

## Check your understanding

{% quiz %}
? What happens to a program with advanced blocks when the switch is off?
- [ ] It won't open
- [x] It still opens and runs
- [ ] The advanced blocks are deleted
> The switch only changes what's in the toolbox.

? What goes in a grey "your Python here" block?
- [ ] A whole program
- [x] One line of Python
- [ ] Only comments
> One line per block. Paste several lines and you get several blocks.

? What does `for note in notes:` do?
- [ ] Plays every note at once
- [x] Runs the loop once for each item in the list
- [ ] Creates a new list
> Each time round, `note` holds the next item from `notes`.
{% endquiz %}
