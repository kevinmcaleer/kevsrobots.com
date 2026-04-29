---
title: Happy 13th Birthday MicroPython!
description: >-
    MicroPython turns 13 today! Let's celebrate with a round-up of the best MicroPython resources on KevsRobots and across the web — then dive into Arduino's brand new MicroPython IDE and see how it stacks up against Thonny.
excerpt: >-
    MicroPython turns 13 today — and what better way to celebrate than with a tour of the best MicroPython resources around, plus a hands-on look at Arduino's new MicroPython IDE, the Arduino MicroPython Installer, and how it all compares to Thonny.
layout: showcase
date: 2026-04-29
author: Kevin McAleer
difficulty: beginner
cover: /assets/img/blog/micropython/cover.jpg
hero: /assets/img/blog/micropython/hero.png
mode: light
tags:
  - micropython
  - birthday
  - arduino
  - thonny
  - ide
  - raspberry pi pico
  - esp32
  - nano esp32
groups:
  - micropython
  - arduino
  - pico
---

Ahoy there makers,

🎉 **Happy 13th Birthday MicroPython!** 🎉

Thirteen years ago, **Damien George** launched a Kickstarter for a strange little board called the **PyBoard**, with an even stranger idea — to run a full Python interpreter on a tiny microcontroller. People backed it. The board shipped. And from those modest beginnings grew **MicroPython** — the language that has, more than any other single thing, made microcontrollers approachable for makers, students, hobbyists and educators all over the world.

It's hard to overstate how much MicroPython has changed the game. Before MicroPython, learning to program a microcontroller meant wrangling C, Makefiles and toolchains. After MicroPython, it means plugging in a board and typing `print("hello")` at a REPL. That difference is the difference between *some people* learning embedded development and *anyone* learning embedded development.

So today, on its 13th birthday, I want to do two things:

1. **Celebrate MicroPython** with a round-up of the best learning resources, courses, projects and links — both here on KevsRobots and out across the wider community.
2. **Take a look at Arduino's MicroPython IDE** — Arduino Lab for MicroPython — and see how it compares to the long-reigning champion, Thonny.

Pour yourself a cuppa. Let's get into it.

---

## 🎂 13 Years of MicroPython — What's Changed

When MicroPython first launched in **2013**, it ran on **one board** — the original PyBoard, a custom STM32-based board Damien designed himself. Today, MicroPython runs on:

- **Raspberry Pi Pico, Pico W, Pico 2 and Pico 2W** (RP2040 and RP2350)
- **ESP32 and ESP8266** in all their many variants
- **Arduino Nano ESP32, Nano RP2040 Connect, Portenta H7, Giga R1**
- **micro:bit** (a tailored MicroPython port)
- **STM32 boards**, **Teensy**, **WiPy**, **OpenMV**, and dozens more

Some of the milestones along the way:

- **2013** — Kickstarter campaign and the original PyBoard
- **2014** — First public MicroPython releases
- **2016** — BBC micro:bit ships with MicroPython, putting it in front of a million UK schoolkids
- **2017** — ESP8266 port lands and explodes the maker user base
- **2021** — Raspberry Pi Pico launches with MicroPython as a first-class language
- **2024** — Pico 2 / RP2350 support, full Pico W networking improvements
- **2026** — Arduino releases their official MicroPython IDE (which we'll come to in a moment!)

Thirteen years in, MicroPython is no longer a clever experiment — it's a proper, dependable, **first-class option** for embedded development. And it's still entirely free and open source.

---

## The Best MicroPython Resources on KevsRobots

Over the years I've built a fair pile of MicroPython content here on the site. If you're learning, here's roughly the order I'd tackle it in.

### Courses

{% include gallery.html titles="How to install MicroPython, MicroPython, MicroPython GPIO, Intermediate MicroPython, MicroPython Robotics, Arduino to MicroPython, I2C and SPI, Pico Temperature Sensor" 
images="
 /learn/how_to_install_micropython/assets/cover.jpg,
 /learn/micropython/assets/micropython.jpg, 
 /learn/micropython_gpio/assets/raspberry_pi_pico_gpio.jpg,
 /learn/intermediate_micropython/assets/cover.png, 
 /learn/micropython_robotics/assets/cover.jpg,
 /learn/arduino_to_python/assets/cover.jpg,
 /learn/i2c_spi/assets/cover.jpg,
 /learn/pico_temp_sensor/assets/cover.jpg"

  links="/learn/how_to_install_micropython/,/learn/micropython/,/learn/micropython_gpio/,/learn/intermediate_micropython/,/learn/micropython_robotics/,/learn/arduino_to_micropython/,/learn/i2c_spi/,/learn/pico_temp_sensor/" descriptions="The absolute first step, the foundation course, GPIO, intermediate topics, robotics, Arduino to MicroPython, I2C and SPI, Pico Temperature Sensor" use-links=true noborder=true%}

---

### Ideas page

- **[All MicroPython content on KevsRobots](/ideas/micropython.html)** — every course, blog post and project I've ever tagged as MicroPython, in one place.

### Recent MicroPython blog posts worth a read

- **[Why are Bus Servos better?](/blog/bus-servos.html)** — controlling Feetech STS3215 bus servos from a Pico in MicroPython.
- **[MicroPython OLED Displays](/blog/micropython-oled-displays.html)** — driving SSD1306 displays in MicroPython.
- **[New Course - Arduino to MicroPython Quick Start](/blog/arduino-to-micropython.html)** — for the Arduino crowd jumping across.

---

## The Best MicroPython Resources Elsewhere

KevsRobots is a great starting point, but the wider MicroPython community is enormous. Here are the resources I genuinely use.

### Official

- **[micropython.org](https://micropython.org)** — the project itself. Downloads, docs, and the wiki.
- **[MicroPython documentation](https://docs.micropython.org/)** — surprisingly readable. Bookmark it.
- **[MicroPython on GitHub](https://github.com/micropython/micropython)** — source code, issues, releases.
- **[MicroPython forum](https://forum.micropython.org/)** — the community Q&A board. Damien himself still posts.
- **[MicroPython Discord](https://discord.gg/RB8HZSAExQ)** — fast, friendly help.

### Learning

- **[Real Python — MicroPython articles](https://realpython.com/micropython-on-rp2/)** — well-written tutorials.
- **[Adafruit Learn — CircuitPython & MicroPython](https://learn.adafruit.com/)** — Adafruit's own teaching content (CircuitPython is MicroPython's sister project).
- **[Raspberry Pi Pico documentation](https://www.raspberrypi.com/documentation/microcontrollers/)** — official Pico/MicroPython guide from the Foundation.
- **[Peter Hinch's MicroPython repos](https://github.com/peterhinch)** — outstanding asyncio guide and dozens of polished libraries.

### Libraries and tools

- **[awesome-micropython](https://github.com/mcauser/awesome-micropython)** — the canonical curated list of MicroPython libraries.
- **[mpremote](https://docs.micropython.org/en/latest/reference/mpremote.html)** — official command-line tool for talking to a board.
- **[mip](https://docs.micropython.org/en/latest/reference/packages.html)** — MicroPython's own package manager.
- **[Pimoroni MicroPython libraries](https://github.com/pimoroni/pimoroni-pico)** — fantastic drivers for their Pico add-ons.

### YouTube channels worth subscribing to

- **[Damien George talks](https://www.youtube.com/results?search_query=damien+george+micropython)** — the man himself, often at PyCon and EuroPython.
- **[Bhavesh Kakwani](https://www.youtube.com/@bhaveshkakwani)** — excellent deep-dive MicroPython videos.
- **[KevsRobots](https://www.youtube.com/@kevinmcaleer28)** — and yes, this one too! 😉

---

## 🎁 The Birthday Present from Arduino

Now — here's the bit that actually inspired this post. To round out MicroPython's 13th birthday year, **Arduino** have given the community a properly nice present: an **official, open source, cross-platform MicroPython IDE**, and a **one-click MicroPython firmware installer**.

If you've been writing MicroPython for any length of time, the chances are you've been doing it in **Thonny**. It's free, it works on every platform, and it's been the de-facto IDE for the Pico community since day one. But there's a new — well, newer — kid on the block, and it comes from a name you'll definitely recognise: **Arduino**.

Arduino Lab for MicroPython is Arduino's free, open-source editor built specifically for MicroPython development. Pair it with the brand new **Arduino MicroPython Installer** and the built-in **Package Installer**, and you've got a complete MicroPython workflow that's polished, fast and refreshingly modern.

This week I've been putting it through its paces — flashing firmware, writing code, installing packages — and comparing it head-to-head against Thonny. Here's what I've found.

---

## What is Arduino Lab for MicroPython?

Arduino Lab for MicroPython is a **lightweight, cross-platform editor** designed from the ground up for MicroPython. It's an Arduino "Lab" project, which means it's officially supported by Arduino but explicitly experimental — they're using it to try out new ideas without committing them to the main Arduino IDE.

It runs on **Windows, macOS and Linux**, and it works with any board that runs MicroPython — not just Arduino boards. Pi Pico, Pico W, Pico 2, ESP32, Arduino Nano ESP32, Portenta H7, Nano RP2040 Connect — they all just work.

Under the hood it's an **Electron app**, which means it's the same kind of tech as VS Code. The UI is clean, modern and feels much more like a contemporary code editor than Thonny does.

---

## What Can It Do?

Out of the box, Arduino Lab for MicroPython gives you everything you need to write, run and manage MicroPython code on a connected board:

- **Code editor** with MicroPython syntax highlighting
- **File browser** showing files on your computer *and* the connected board, side by side
- **Interactive REPL** at the bottom of the screen for trying things out live
- **Run on Board** — runs the current file directly on the board without uploading first
- **Upload to Board** — copies your file across so it survives a reboot
- **`main.py` runner** — runs the board's `main.py` directly from the editor
- **Soft reset** and **stop** buttons that actually do what you'd expect
- **Drag-and-drop file copy** between your computer and the board's filesystem

That last one is genuinely lovely. Want to copy `boot.py` and `main.py` and a `lib/` folder onto the Pico? Drag them across. No `mpremote` incantations, no copy-and-paste-each-file-individually like in Thonny.

---

## Installing MicroPython — the Easy Way

Before you can do *anything*, your board needs to be running MicroPython firmware. Historically this meant:

1. Going to micropython.org
2. Working out which `.uf2` or `.bin` file you needed
3. Putting your board into bootloader mode the right way
4. Dragging the file across (or using `esptool` and praying)

Arduino's solution is the **Arduino MicroPython Installer** — a separate tiny standalone tool that handles the lot for you.

You plug in the board, the installer detects it, you click **Install MicroPython**, and it does the entire dance for you — picks the correct firmware version, puts the board into bootloader mode, flashes the file, and verifies. Job done in about 30 seconds.

Supported boards include:

- **Arduino Nano ESP32**
- **Arduino Nano RP2040 Connect**
- **Arduino Portenta H7**
- **Arduino Giga R1 WiFi**
- **Raspberry Pi Pico / Pico W / Pico 2**
- **ESP32 dev boards**

If you've ever tried to talk a beginner through flashing firmware over the phone, you'll appreciate just how big a deal this is. **One click. One tool. Done.**

---

## The Built-In Package Installer

The other really nice addition is the **Package Installer** baked into Arduino Lab for MicroPython itself.

In normal MicroPython land, you install packages with `mip` — MicroPython's package manager — usually by typing something like:

```python
import mip
mip.install("github:peterhinch/micropython-async")
```

…over the REPL. Functional, but clunky.

In Arduino Lab, there's a **package browser**. You search for what you want — `micropython-async`, `umqtt.simple`, `urequests` — click **Install**, and it pulls the package down onto the board into the right folder structure. No REPL juggling, no path drama.

For beginners this is huge. Discovering libraries, installing them, and using them is now a three-click affair.

---

## Arduino Lab for MicroPython vs Thonny

So here's the question every Pico-toting maker is going to ask — **should I switch from Thonny?**

Let's do a proper side-by-side.

| Feature | Thonny | Arduino Lab for MicroPython |
|---|---|---|
| Cost | Free | Free |
| Platforms | Win / Mac / Linux | Win / Mac / Linux |
| Code editor | Basic, functional | Modern, VS Code-style |
| File explorer | Two-pane (PC + board) | Two-pane (PC + board) |
| REPL / Shell | Yes | Yes |
| Plotter | Yes (built-in) | No |
| Variable inspector | Yes | No |
| Step-through debugger | Yes (Python only) | No |
| Firmware flasher | Yes (manual board picks) | Via separate Arduino MicroPython Installer |
| Package installer | Manual (`mip` over REPL) | Built-in package browser |
| Drag-and-drop file copy | Limited | Yes |
| Board support | Huge — anything MicroPython runs on | Anything MicroPython runs on |
| Project / multi-file workflow | Basic | Better — folders, lib/, modular |
| Active development | Steady | Very active, lots of new features |
{:class="table table-single table-narrow"}

### Where Thonny still wins

- **Plotter and variable inspector.** If you're teaching a class and you want to plot sensor values live, Thonny's built-in plotter is brilliant and Arduino Lab simply hasn't got one yet.
- **Debugger.** Thonny has a proper step-through debugger for plain Python code. It's not the most amazing thing in the world, but it's there.
- **Maturity.** Thonny has been around for years. It's stable, predictable, and very widely documented. If you Google a problem, the answer is on a forum somewhere.
- **Lower-spec machines.** Thonny is lighter than an Electron app. On an old laptop or a Raspberry Pi desktop, Thonny will feel snappier.

### Where Arduino Lab pulls ahead

- **Modern UI.** It just looks and feels like a 2026 code editor. Tabs, themes, sensible keyboard shortcuts.
- **Package installer.** This is the killer feature. Adding `urequests` shouldn't require a REPL paste.
- **Firmware installer.** The Arduino MicroPython Installer takes one of the most off-putting parts of starting MicroPython — flashing the firmware — and just removes it.
- **Better file management.** Drag-and-drop, folder support, `lib/` workflow all feel more natural for projects that go beyond a single `main.py`.
- **Cleaner REPL.** The terminal area is properly separated from the editor and behaves like a real terminal.

### My recommendation

- **Brand new to MicroPython?** Start with **Arduino Lab for MicroPython**. The firmware installer alone is worth it, and the package browser will save you hours of frustration learning `mip` syntax.
- **Teaching kids or running a workshop?** **Thonny** still wins, mostly because of the plotter and the sheer volume of existing tutorials that point at it.
- **Day-to-day MicroPython project work?** Honestly — **Arduino Lab**. It's nicer to live in.
- **Already happy in Thonny?** No reason to switch unless something on this list grabs you. They both do the job.

I've started using **Arduino Lab for MicroPython** for new projects and falling back to Thonny when I want the plotter. The two coexist on my machine just fine — they're not mutually exclusive.

---

## Getting Started

Here's how to get up and running in about two minutes.

### 1. Install Arduino Lab for MicroPython

Download from the official Arduino Lab site:

- <https://labs.arduino.cc/en/labs/micropython>

There are builds for Windows, macOS (Intel and Apple Silicon) and Linux.

### 2. Install the Arduino MicroPython Installer

Grab the standalone installer from:

- <https://labs.arduino.cc/en/labs/micropython-installer>

Run it, plug in your board (Pico, Nano ESP32, whatever you've got), click **Install MicroPython**, and wait 30 seconds.

### 3. Open Arduino Lab and connect

Launch Arduino Lab for MicroPython, click the **Connect** button, pick your board's serial port from the dropdown, and you're in. The REPL will print the MicroPython prompt:

```
MicroPython v1.23 on 2026-XX-XX; Raspberry Pi Pico W with RP2040
Type "help()" for more information.
>>>
```

### 4. Hello, blinky

Type this into the editor:

```python
from machine import Pin
import time

led = Pin("LED", Pin.OUT)

while True:
    led.toggle()
    time.sleep(0.5)
```

Hit **Run on Board**. The onboard LED starts blinking. Welcome to MicroPython, the Arduino way.

### 5. Install a package

Open the **Package Installer**, search for `urequests`, click **Install**. That's it — you've now got HTTP request capability on your board, no REPL mip-pasting needed.

```python
import urequests
r = urequests.get("https://api.github.com")
print(r.status_code)
r.close()
```

---

## Things to Watch Out For

A few rough edges I've hit while using it day-to-day:

- **It's a Lab project.** Arduino are clear that this is experimental. Things change between versions. Don't rely on it for mission-critical workflows just yet.
- **No plotter.** If you want to graph sensor values, you'll need to fall back to Thonny or a separate tool.
- **Package installer doesn't show every package.** It pulls from `micropython-lib` and a curated list. If you need something obscure, you'll still want to know how `mip` works under the hood.
- **Electron means RAM.** The app uses a few hundred MB of memory at idle — not a problem on a modern machine, noticeable on a Pi 4 desktop.
- **One serial port at a time.** Like Thonny, you can't have two boards open in two windows easily. That's a MicroPython tooling thing in general.

None of these are deal-breakers. They're just things to know.

---

## Verdict

Arduino Lab for MicroPython is **the polished, modern MicroPython IDE the community has been quietly waiting for**. It's free, it's cross-platform, it works with non-Arduino boards, and the firmware installer + package installer combo removes two of the biggest friction points beginners hit.

Thonny isn't going anywhere — it's still the right choice for classrooms and for the plotter — but for everyday MicroPython project work, Arduino Lab feels like the future.

Give the **Arduino MicroPython Installer** a download and try flashing your Pico in 30 seconds. Even if you stick with Thonny afterwards, that one tool alone is going to save you a real amount of pain.

---

## Links

- **Arduino Lab for MicroPython:** <https://labs.arduino.cc/en/labs/micropython>
- **Arduino MicroPython Installer:** <https://labs.arduino.cc/en/labs/micropython-installer>
- **Thonny:** <https://thonny.org>
- **MicroPython:** <https://micropython.org>
- **MicroPython `mip` package manager:** <https://docs.micropython.org/en/latest/reference/packages.html>
- **Arduino to MicroPython course:** [/learn/arduino_to_python/](/learn/arduino_to_python/)

---

If you're new to MicroPython and you've been put off by firmware flashing or `mip` install commands, **Arduino Lab for MicroPython is the easiest on-ramp I've seen**. Give it a try, and let me know in the comments how you get on.

---

## 🎂 Happy Birthday, MicroPython

Thirteen years on, MicroPython is still doing what it set out to do — making microcontrollers accessible to everyone. Whether you came in via a PyBoard, a micro:bit, an ESP8266, a Raspberry Pi Pico or — soon — an Arduino Nano ESP32 with the new Arduino MicroPython Installer, **welcome to one of the friendliest communities in tech**.

If you've never written any MicroPython, today is a perfect day to start. Pick up a Pico, head over to **[How to Install MicroPython](/learn/how_to_install_micropython/)**, and write your first `print("hello")` over the REPL. You'll be hooked within an hour.

A massive thank you to **Damien George** and the entire MicroPython team and community for thirteen years of incredible work. Here's to the next thirteen. 🐍🎉

If you found this useful, give the video a thumbs up — it really does help the channel.

See you next time. Bye for now.

---
