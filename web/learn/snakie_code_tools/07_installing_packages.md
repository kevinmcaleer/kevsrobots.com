---
layout: lesson
title: Installing Packages
author: Kevin McAleer
type: page
cover: /learn/snakie_code_tools/assets/banners/07_installing_packages.jpg
date: 2026-11-21
previous: 06_yaml_and_json.html
next: 08_the_modules_shelf.html
description: Find and install MicroPython libraries onto your board with mip from
  Snakie's Packages panel, and manage what is already there
percent: 48
duration: 5
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

Half of programming is not writing code that someone else has already written. MicroPython libraries are called **packages**, and the tool that installs them is **mip**. Snakie puts a friendly face on it.

---

## The Packages panel

Click **Packages** in the activity bar on the left.

![The Packages panel with the board's libraries and the registry](/learn/snakie_code_tools/assets/packages.jpg){:class="img-fluid w-100"}

From top to bottom:

| Part | What it shows |
|---|---|
| **Packages** and **Modules** tabs | This lesson, and the next one |
| **FLASH** meter | How much of the board's storage is used |
| **Search** | Find a package by name |
| **Advanced options** | Install from somewhere other than the standard index |
| **ON BOARD (/LIB)** | What's installed now, with version numbers and an **Uninstall** button |
| **REGISTRY** | Packages you can install, each with an **INSTALL** button |
{:class="table table-single"}

Packages marked **CURATED** are a short list of well-known, widely used libraries: `urequests` for web requests, `umqtt.simple` for MQTT, `ssd1306` for OLED displays, `neopixel`, `bme280`, `dht` and more.

## Installing one

1. **Connect a board.** Installing copies files onto it, so there has to be one. The INSTALL buttons say *Connect a board first* until there is
2. **Search.** Type `ssd1306` and press **Search**
3. Click **INSTALL**

Snakie runs mip for you and shows the progress. The files land in `/lib` on the board, which MicroPython searches automatically, so your program can use it straight away:

```python
import ssd1306
```

The package then appears under **ON BOARD (/LIB)**.

## Boards without Wi-Fi

mip normally runs *on the board* and downloads over the board's own internet connection. A plain Pico has no Wi-Fi, so that can't work.

Snakie handles it. It downloads the package on your computer and copies the files across the USB cable. From where you sit, INSTALL works the same on every board.

## What does my program need?

When you've opened a project folder, the panel can read your code for you. **Scan imports** looks through the project's `.py` files, lists every `import` that nothing on the board or in your folder satisfies, and offers to install each one.

It's the quickest fix for the most common error of all:

```
ImportError: no module named 'ssd1306'
```

## Parts bring their own drivers

If you wire up a part in the Electronics workspace that needs a driver, Snakie offers it in a banner above the editor: *1 part needs a driver*. That's the same machinery, started from the other end. The [Electronics course](/learn/snakie_electronics/07_drivers_help_and_code.html) covers it.

## Advanced options

Open **Advanced options** to install something that isn't in the list:

- A package from GitHub, written as `github:user/repo`
- A package from a different index, by giving its URL
- Whether to overwrite files that are already there

Only install from sources you trust. A package is code that runs on your board with full access to its pins.

## Keeping the board tidy

Flash storage is small. A Pico has about 1.4 MB for files. Watch the **FLASH** meter, and use **Uninstall** on libraries from old experiments. Each row's button tells you exactly which file it will remove, such as `/lib/turtle.py`.

---

## Try it Yourself

1. Open Packages with the simulated device connected. What's already in `/lib`?
2. Search for `mqtt`. How do the two results differ, according to their descriptions?
3. Find a package for a sensor you own. Read its description before you install it.

## Common Issues

**Problem**: The INSTALL buttons are greyed out.
**Solution**: Connect a board, or the simulated device.
**Why**: Packages are installed onto a board's file system.

**Problem**: The install fails part-way.
**Solution**: Check your computer is online, and that the board has free space on the FLASH meter.
**Why**: Snakie downloads the files first, then copies them over. Either step can fail.

**Problem**: I installed a package but `import` still fails.
**Solution**: Check the name you import matches the module's file name in `/lib`, and press **Reset** on the board.
**Why**: The package name in the registry and the module name you import are sometimes different.

## Check your understanding

{% quiz %}
? Where do installed packages go on the board?
- [ ] /packages
- [x] /lib
- [ ] The firmware
> MicroPython looks in /lib automatically when you import.

? How does Snakie install onto a board with no Wi-Fi?
- [ ] It can't
- [ ] It asks you to add a Wi-Fi module
- [x] It downloads on your computer and copies the files over USB
> So installing works the same way on every board.

? What does Scan imports do?
- [x] Finds imports in your project that nothing satisfies
- [ ] Deletes unused libraries
- [ ] Updates the firmware
> It turns an ImportError you haven't had yet into a one-click install.
{% endquiz %}
