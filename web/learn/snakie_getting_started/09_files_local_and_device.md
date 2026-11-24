---
layout: lesson
title: Files on Your Computer and on the Board
author: Kevin McAleer
type: page
cover: /learn/snakie_getting_started/assets/banners/09_files_local_and_device.jpg
date: 2026-11-20
previous: 08_connect_run_stop.html
next: 10_project_traffic_light.html
description: Save MicroPython programs to your computer and to your board in Snakie,
  and make one run at power-up with main.py
percent: 80
duration: 5
quiz_total: 23
date_updated: 2026-11-24
navigation:
- name: Getting Started with Snakie
- content:
  - section: Overview
    content:
    - name: Getting Started with Snakie
      link: 00_intro.html
    - name: Web App or Desktop App
      link: 01_web_or_desktop.html
    - name: A Tour of the Workspaces
      link: 02_tour_of_the_workspaces.html
  - section: Your First Programs
    content:
    - name: Your First Program on the Simulator
      link: 03_first_program_on_the_simulator.html
    - name: The REPL and the Console
      link: 04_the_repl_and_console.html
    - name: Seeing Your Pins
      link: 05_seeing_your_pins.html
  - section: Real Hardware
    content:
    - name: Finding Your Board
      link: 06_finding_your_board.html
    - name: Flashing MicroPython
      link: 07_flashing_micropython.html
    - name: Connect, Run and Stop
      link: 08_connect_run_stop.html
    - name: Files on Your Computer and on the Board
      link: 09_files_local_and_device.html
  - section: Project
    content:
    - name: Project - Traffic Light
      link: 10_project_traffic_light.html
    - name: Summary
      link: 11_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to save a program on your computer and on your board, and make it run at power-up as main.py.

A robot that only works while it's plugged into your laptop isn't much of a robot. In this lesson you'll save your program onto the board so it runs by itself.

---

## Two sets of files

Open the **Files** panel with the folder icon in the side bar. It has two halves:

- **Local files** are on your computer. Click **Open Folder** and choose a folder to keep your project in
- **Device files** are on the board. A new board has very little there. You may see a `lib` folder, which is where libraries live

Between them are two arrows. The down arrow uploads the file you're editing to the board. The up arrow downloads a file from the board to your computer.

Keep the master copy of your code on your computer. A board can be wiped, lost or sat on.

## Save to your computer

Press the **Save** button in the toolbar, or <kbd>Ctrl</kbd>+<kbd>S</kbd>. The first time, Snakie asks where to put the file and what to call it. Give it a name that says what it does, such as `traffic_light.py`.

The tab shows a dot when a file has changes you haven't saved, and the status bar says **Unsaved** or **Saved**.

## Upload to the board

With your program open:

1. Click the down arrow between the two file lists.
2. Snakie asks for the **device path**. Type `/main.py` and click **OK**.

![Uploading a file to the board](/learn/snakie_getting_started/assets/upload_dialog.jpg){:class="img-fluid w-100"}

3. Click **Refresh** in the Device files toolbar. `main.py` appears in the list, with its size and a **runs at boot** tag.

![main.py on the device](/learn/snakie_getting_started/assets/device_files.jpg){:class="img-fluid w-100"}

## Why main.py?

When a MicroPython board starts, it looks for a file called `main.py` and runs it. That one filename is the difference between a program you run from Snakie and a program the board runs by itself.

Press **Reset**, or unplug the board and plug it into a USB power bank. Your program starts with no computer involved.

| File on the board | When it runs |
|---|---|
| `boot.py` | First, at every start. Used for setup; most projects don't need one |
| `main.py` | Straight after, at every start. This is your program |
| Anything else | Only when your code imports it |
{:class="table table-single"}

## Working with device files

Click a file in **Device files** to open it. Its tab name appears in `[brackets]` so you can tell it from the copy on your computer. Right-click a device file to rename it, delete it or download it.

The gauge in the Device files toolbar shows how much of the board's storage is used. An original Pico has roughly 1.4 MB for files once MicroPython is installed, which is plenty for code but not for big data files.

## Keep files in sync

Uploading by hand after every change gets old quickly. Snakie can do it for you:

1. Open a folder in **Local files**.
2. Hover over a file and tick its checkbox. A green sync mark shows it's tagged.
3. Turn on the sync toggle in the Device files toolbar.

Now every tagged file is copied to the board each time you save it. Save, press **Reset**, and you're running the new version.

## Try it on the simulator

No board? Everything in this lesson works on the simulated device too. It has a real filesystem in memory, so uploads, downloads and sync behave the same way. It's emptied when you disconnect.

---

> **Teachers**: On shared computers, keep each learner's master copy in their own folder on the computer, because a board can be wiped or lost. If boards are shared between classes, each new learner will find the previous learner's main.py running, so agree who overwrites it. Only one program can use a board's serial port at a time.

## Try it Yourself

1. Upload the blink program from the last lesson as `/main.py` and press **Reset**.
2. Upload a second file called `/settings.py` containing `DELAY = 0.2`.
3. Challenge: change `main.py` to start with `from settings import DELAY` and use `DELAY` in `time.sleep()`. You've just made your first module.

## Common Issues

- **Problem**: I uploaded the file but it isn't in the list.
- **Solution**: Click **Refresh** in the Device files toolbar.
- **Why**: The list isn't re-read after every upload.

- **Problem**: My program doesn't start at power-up.
- **Solution**: Check the file is named exactly `main.py`, in lower case, at the top level of the board and not in a folder.
- **Why**: MicroPython only looks for that exact name in that exact place.

- **Problem**: I can't connect because `main.py` is stuck in a loop.
- **Solution**: Connect anyway and press **Stop**. Snakie interrupts the program so you can get back in.
- **Why**: A running `main.py` keeps the board busy until it's interrupted.

## Check your understanding

{% quiz %}
? What must a file be called to run automatically when the board powers up?
- [ ] start.py
- [x] main.py
- [ ] run.py
> MicroPython runs `main.py` at every start.

? Where should you keep the master copy of your code?
- [x] On your computer
- [ ] Only on the board
- [ ] In the console
> Boards get wiped and lost. Keep your code on your computer and upload copies.
{% endquiz %}
