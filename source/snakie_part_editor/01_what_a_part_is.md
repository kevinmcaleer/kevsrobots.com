---
title: What a Part Is
description: Understand how Snakie stores a part as a folder with a parts.yml file, and how parts are grouped into libraries
layout: lesson
type: page
cover: /learn/snakie_part_editor/assets/banners/01_what_a_part_is.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to describe what a Snakie part is, name the files in a part folder, and find your parts folder on your computer.

Before you open the editor, it helps to know what you're making. A Snakie part isn't hidden inside the app. It's a **folder of plain files** that you could open in any text editor.

---

## One folder per part

Here's the folder for the part you're about to build:

```
vl53l0x-distance-sensor/
  parts.yml      the part itself: size, pins, holes, drivers
  image.png      the board photo (optional)
  help.md        bundled help (optional)
  blocks.yml     the part's own blocks (optional)
```

Only `parts.yml` is required. The Part Editor writes it for you, so you rarely type it by hand, but it's worth reading one once. This is the start of the file you'll end up with:

```yaml
id: vl53l0x-distance-sensor
name: VL53L0X Distance Sensor
description: Time-of-flight laser distance sensor on an I2C breakout. Measures up to 2 m.
manufacturer: STMicroelectronics
family: Sensor
tags:
  - i2c
  - distance
  - tof
pinSpacing: 2.54
voltage: 2.6-5.5V
dimensions:
  width: 25
  height: 13
headers:
  - edge: left
    pins:
      - name: VIN
        type: pwr
        number: 1
        shape: round
        x: 0.246
        y: 0.8615
```

Read it top to bottom and it describes a real object: a 25 by 13 mm sensor board whose first pin is a power pin called VIN.

> Positions such as `x` and `y` are **fractions of the board**, not millimetres. `x: 0` is the left edge and `x: 1` is the right edge. `y: 0` is the top and `y: 1` is the bottom. A part keeps its shape when it's drawn at any size.

---

## Parts live in libraries

A **library** is a folder of part folders, with a small `library.yml` file that names it.

```
parts/
  snakie-standard/          the Standard Parts that ship with Snakie
    library.yml
    bme280/
    hc-sr04/
    ...
  my-parts/                 everything you make
    library.yml
    vl53l0x-distance-sensor/
```

The first time you save a part, Snakie creates the **My Parts** library for you. Its `library.yml` is four lines long:

```yaml
id: my-parts
name: My Parts
description: Parts you authored in the Part Editor.
version: 0.1.0
```

| File | What it describes | Who writes it |
|---|---|---|
| `library.yml` | A whole library: its name, author and version | Snakie, or you when you share one |
| `parts.yml` | One part | The Part Editor |
| `image.png` | The picture drawn on the board | The Part Editor, from your photo |
| `help.md` | Help shown when the part is used | The Part Editor, from the Mini-help box |
| `blocks.yml` | Blocks the part adds to the Blocks workspace | You |
{:class="table table-single"}

---

## Finding the folder

1. Open the **Electronics** workspace.
2. In the **Library** panel on the right, click the folder button, **Reveal the parts folder**.

Your file manager opens at the parts folder. On Linux it is `~/.config/Snakie/parts`. On Windows and macOS it's in Snakie's application data folder, and the button takes you straight there.

> **Teachers**: Parts are saved as plain files in a folder on the computer, so on shared machines each learner's parts end up in the same place unless they use separate user accounts. Ask learners to give their parts distinct names, and to copy their part folder to their own storage before they leave.

---

## Try it Yourself

1. Reveal the parts folder and open `snakie-standard`.
2. Open the `bme280` folder. It has help, blocks and its own driver file, so it's a great example.
3. Open its `parts.yml` in a text editor. Find the pin called **SDA** and read its `capabilities`.
4. Close the file without saving.

---

## Common Issues

**Problem**: There's no `my-parts` folder.
**Solution**: Nothing is wrong. It appears the first time you save a part.
**Why**: Snakie only creates your library when it has something to put in it.

**Problem**: I edited a file in `snakie-standard` and my change vanished.
**Solution**: Make your own copy with **Duplicate part** and edit that.
**Why**: Snakie keeps the Standard Parts up to date, and refreshes parts you haven't changed through the editor.

---

## Check your understanding

{% quiz %}
? Which file must every part folder contain?
- [x] parts.yml
- [ ] blocks.yml
- [ ] image.png
> The picture, the help and the blocks are all optional extras.

? A pin has x: 0.5 and y: 1. Where is it?
- [ ] In the top left corner
- [x] In the middle of the bottom edge
- [ ] Half a millimetre from the left
> Positions are fractions of the board, so 0.5 across and all the way down.

? Where do parts you make get saved?
- [ ] Inside the Standard Parts library
- [x] In a library called My Parts
- [ ] In your project folder
> My Parts is created automatically on your first save.
{% endquiz %}
