---
title: "What If Building Robots With Code Was Actually Easy?"
description: >-
  Meet Snakie — a free, open-source, robotics-first MicroPython editor with Monaco editing, built-in git, an LLM chat pane, live sensor instruments, and a board viewer that hands you driver code just by dragging a part in.
excerpt: >-
  What would a code editor look like if it was actually designed around building robots, instead of just editing text? I built the answer, and Adafruit noticed too.
layout: showcase
mode: light
date: 2026-07-03
author: Kevin McAleer
difficulty: beginner
cover: /assets/img/blog/snakie/cover.jpg
hero: /assets/img/blog/snakie/hero.png
tags:
  - micropython
  - raspberry pi pico
  - snakie
  - code editor
  - ide
  - monaco editor
  - robotics
  - open source
  - developer tools
groups:
  - micropython
  - pico
  - ai
videos:
  - QVe9meIHZtk
  - qfT_Ndq-tJI
code:
  - https://www.snakie.org
---

Ahoy there Makers,

Here's a question I keep coming back to: if building robots with code was actually easy, what would that even look like? What does an IDE even look like, in a post-code era?

Well — I built the answer. It's called Snakie, and this post is the full, standalone write-up: the editor itself, the features, the real code you'll actually type, the parts on my desk while I filmed the demo, and the gotchas that didn't fit in the video. If you want the tour with sound effects and a live drone wobble, but everything you need to actually go and use this lives here.

---

## The one peeve that started it

Quick bit of context before Snakie itself. My day-to-day MicroPython workflow used to mean switching between **VS Code** and **Thonny** — one's the nicer editor to actually write in, the other's the better tool for getting code onto a microcontroller. Depending what I was doing, I'd bounce between the two, sometimes in the same afternoon. Add in forgetting which pin's which halfway through wiring something up, and hunting round the internet for the right driver for whatever part I'd just bought, and you start to see the shape of the problem.

So I started wondering: what if there was a code editor that was actually robotics-first? Not a text box that happens to run Python, but something built around the fact that you're wiring up sensors and motors, not just writing lines of code.

So I built one.

---

## Six months at a desk, then a public repo

This is the same story as a lot of tools you already use. **Linus Torvalds wrote git in about ten days, in 2005**, because none of the existing tools did what he needed. **Redis exists because a developer called antirez needed something MySQL simply couldn't do fast enough** for the real-time analytics startup he was running. So many of the tools you use every single day started exactly like that — one person, annoyed enough, building the thing they wished existed.

I just did it for MicroPython.

Snakie spent about six months as a private, at-my-desk prototype before I ever pushed it public — the public GitHub repo only went live at the start of June 2026, and it's been shipping new features at a genuinely fast clip ever since (it's already on v0.21.0 as I write this). Six months ago this was a question I was asking myself at this desk. Now it's just the thing I build robots in, every single day.

---

## Meet Snakie

Snakie is a free, [MIT-licensed](https://www.snakie.org), cross-platform MicroPython editor — it runs on Windows, macOS and Linux from a single Electron codebase, the same approach VS Code, Slack and Discord all use. On top of that, it's robotics-first: git, an LLM, board visualisation, and parts help are all built in, not bolted on.

---

### The editor: Monaco underneath

The text-editing core of Snakie is **Monaco** — the professional code-editor component **Microsoft** builds straight from the VS Code source and ships as a standalone, open-source piece. It's the same engine behind **GitHub Codespaces** and **StackBlitz**. That's not "feels a bit like VS Code" — it's literally the same editor engine, with proper syntax highlighting, sensible autocomplete, code folding, multi-cursor editing and a diff view, out of the box.

Because there's an LLM sitting right there in the background, Snakie also suggests the next line as ghost text while you type — start writing a line to read a sensor, and it's already guessing where you're going with it.

---

### Git, built straight in

Right next to the editor there's proper git source control — no separate app, no separate habit to learn. Make a change, and there's the diff, right there in the same window. Commit, and you're done.

---

### An LLM that actually looks at your code

There's a chat pane built in too, and it can answer questions about the actual code on your screen — something like "why does this reading keep jumping around so much" gets an answer that references the exact function you're looking at, not a generic snippet pulled off the internet.

---

### Keeping your board in sync

The file on your computer and the copy living on the board stay in sync automatically — edit locally, and the version on the device catches up, with no copying back and forth by hand. This is one of Snakie's newest features, so if you haven't updated in a while, it's worth grabbing the latest release for it (more on versions in the gotchas below).

---

### The instrument panel — seeing the feedback loop

This is the part of Snakie I'm most proud of. Robotics comes down to one line: **sensors detect, the brain decides, actuators move.** The genuinely hard part has always been seeing what's actually happening in that loop, while it's happening — normally that means writing throwaway `print()` statements just to find out what a sensor is really reporting.

Snakie's instrument dock makes that loop visible with zero extra code:

- **Rangefinder** — wave a hand in front of it, watch a live number tick upward in real time.
- **IMU** — tilt a drone gently, and the on-screen readout responds instantly.
- **Multimeter** — touch a probe to a pin on the breadboard, and the on-screen ADC voltage matches it, near enough exactly, in real time.
- **Buzzer** — a short tone, or a full little melody, played straight from the board.

No separate app open on the side, no guessing.

---

### The board, breadboard and schematic viewer

Flip over to the board viewer and you get your current build in breadboard view — every wire, every connection, drawn straight from the code you've actually written. Flip it to schematic view and it's the same circuit, properly drawn out. Need it for a build log, or to send to someone else? Export either view straight to PDF or PNG. Documentation that basically draws itself.

---

### The star move: drag a part in

This is the moment that still gives me a genuine buzz every time I do it. Drag a part icon straight onto the breadboard canvas, and Snakie tries to recognise what it is. When it does, a banner appears offering to install that part's driver — **one click**, and the driver code and a wiring guide land in your editor. It's consent-based, not silent — you see the banner, you click it, and only then does anything get written. I didn't open a datasheet, I didn't go hunting round for a library. I dragged a part in and clicked once.

---

## The kind of code you'll actually write

Snakie doesn't invent a new dialect of Python — everything you write is ordinary MicroPython, running directly on the chip with no compile step, which is exactly why the edit-run loop can be seconds rather than minutes. Here's roughly the shape of what you'd type for two of the instrument-dock demos above.

Reading an analog sensor like a rangefinder, so it shows up live on the instrument dock:

```python
from machine import ADC, Pin
import time

rangefinder = ADC(Pin(26))   # analog input pin

while True:
    reading = rangefinder.read_u16()
    voltage = reading * 3.3 / 65535
    print(voltage)
    time.sleep(0.1)
```

Driving the buzzer for that little melody:

```python
from machine import Pin, PWM
import time

buzzer = PWM(Pin(15))
notes = [262, 294, 330, 349]  # a simple C-D-E-F run

for note in notes:
    buzzer.freq(note)
    buzzer.duty_u16(1000)
    time.sleep(0.2)
buzzer.duty_u16(0)
```

Under the hood, Snakie talks to the board over its USB serial connection — the Pico enumerates as a CDC virtual serial port (`/dev/ttyACM0` on Linux, a `COM` port on Windows, `/dev/cu.usbmodem…` on Mac) — using MicroPython's REPL modes: normal interactive mode, paste mode, and raw REPL/raw-paste mode for reliably pushing a whole file to the board. That's the machinery a one-click "flash & run" button is hiding from you.

---

## Parts list — what's on the desk

The demo hardware in the video, for anyone who wants to follow along:

- **Raspberry Pi Pico (or Pico W)** — the microcontroller running MicroPython — {{confirm link}}
- **Ultrasonic rangefinder** — the sensor pinged live on the instrument dock — {{confirm link}}
- **Small drone with an onboard IMU** — used to demo the tilt-responsive instrument readout — {{confirm link}}
- **Multimeter** — to verify the on-screen ADC voltage against a real probe reading — {{confirm link}}
- **Buzzer** — for the tone and melody demo — {{confirm link}}
- **Breadboard and jumper wires** — for the board/breadboard/schematic viewer demo — {{confirm link}}
- **Snakie itself** — free, MIT-licensed, cross-platform — [github.com/kevinmcaleer/Snakie](https://github.com/kevinmcaleer/Snakie)

*(Kevin: the exact rangefinder/drone/multimeter/buzzer models weren't specified in the script or research notes — drop the real product links in here before this goes live.)*

---

## Gotchas — what the video didn't have time for

A few things worth knowing if you're going to actually install and use Snakie, rather than just watch it:

- **The drag-a-part move can take two tries.** In the video, the first drag onto the breadboard canvas didn't register — second time worked cleanly. I left that in on purpose rather than re-shooting to hide it. If you hit the same hiccup, try the drag again before assuming something's broken, and tell me in the comments if you find a repeatable cause — that's exactly the kind of thing I'll take a fix for into a future episode.
- **The driver install is one click, not silent.** When Snakie recognises a dragged-in part, it shows a consent banner first — nothing gets written to your project until you click. That's a deliberate safety choice, not a missing feature.
- **Check your version before you go looking for a feature.** Snakie ships fast. Git and the LLM chat pane have been there since the very first public release (v0.1.0). The board visualiser arrived a few weeks later, and the drag-a-part parts library later still. If you tried an early build and didn't see these, it's worth pulling the [latest release](https://github.com/kevinmcaleer/Snakie/blob/master/CHANGELOG.md) — file auto-sync in particular is a very recent addition.
- **"No port found" errors, if you've fought this before, usually come down to one of a few nameable causes**: a power-only USB cable with no data lines, the board sitting in bootloader mode (which deliberately exposes a mass-storage drive instead of a serial port), Linux permissions (you're not in the `dialout` group), or a crashed `main.py` locking up the board. Snakie's auto-detect handles the common cases, but it's worth knowing what's actually going on underneath if you ever do get stuck.

---

## Adafruit noticed

A couple of weeks before this article went up, something happened that I still don't quite believe: **Adafruit wrote about Snakie.** Their words, not mine:

> "Snakie is a clean, uncluttered IDE for writing MicroPython code and working with connected MicroPython devices."

I've read that sentence back more times than I'd like to admit.

## Try it, break it, tell me what's missing

Coding has changed a lot these last few years. Hardware really hasn't. Snakie is my attempt to close that gap.

It's free. It's open source, MIT licence, and it runs on Windows, macOS and Linux — grab it from [github.com/kevinmcaleer/Snakie](https://github.com/kevinmcaleer/Snakie).

Tell me what breaks. Tell me what's missing. And tell me in the comments — on the video, or right here — what's the one feature that would actually make you switch? If you spot a fix for the drag-and-drop hiccup above, that's a genuine open invitation: the best community fix gets built into a future episode.

Watch the full demo — the drone tilt, the multimeter side-by-side, the live drag-a-part moment. If this is even close to what an IDE should look like in a post-code era, I want to keep building it, with you. Subscribe if you want to see where it goes.

---
