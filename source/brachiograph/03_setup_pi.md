---
title: Setup Raspberry Pi
description: Install Raspberry Pi OS, the pigpio service, a Python virtual environment and the BrachioGraph library.
layout: lesson
type: page
cover: /learn/brachiograph/assets/banners/03_setup_pi.jpg
date_updated: 2026-11-24
snakie: false
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to install the BrachioGraph library on a Raspberry Pi, start the pigpio service, and check that Python can import the library.

## What we are installing, and why

Two things sit between your Python code and the servos.

```mermaid
flowchart LR
    CODE["Your Python code"] --> LIB["BrachioGraph library"]
    LIB --> CLIENT["pigpio Python module"]
    CLIENT --> DAEMON["pigpiod service running on the Pi"]
    DAEMON --> PINS["GPIO 14, 15 and 18"]
```

The **BrachioGraph library** works out the angles. The **pigpio daemon** (`pigpiod`) is a background service that generates the precise servo pulses on the GPIO pins. If the daemon isn't running, the library cannot connect to it, and the official docs note it then falls back to a "virtual" plotter that moves nothing.

> **Teachers**: Installing needs an internet connection and a user who can use `sudo`, which can be blocked on school networks or locked-down machines. Test the install on one Pi before the lesson, and consider preparing one SD card image and cloning it for every group. Each Pi needs its own SD card, so label them per group.

---

## Which Pi and which OS?

The official [install page](https://www.brachiograph.art/tutorial/install/) says the Raspberry Pi OS steps were tested with Raspberry Pi OS **Bullseye Lite**, and it needs Python 3.6 or later. It does not name specific Pi models, and the original version of this course used a Pi 3+. We could not confirm from the pigpio or BrachioGraph sources how the `pigpiod` package behaves on newer Pi models and newer OS releases. If your Pi is recent, check the pigpio project and [brachiograph.art](https://www.brachiograph.art/) before you start, and be ready to fall back on the virtual plotter in the next lessons.

---

## Step 1: Put Raspberry Pi OS on the SD card

1. Download Raspberry Pi Imager from [raspberrypi.com/software](https://www.raspberrypi.com/software/) and install it on your computer.
2. Choose your Pi model, the operating system, and your microSD card.
3. Write the card, put it in the Pi, then connect a screen, keyboard and power. If you set up Wi-Fi and SSH in the Imager's settings you can work over SSH instead.

---

## Step 2: Update and install the system packages

Open a terminal on the Pi. The official Raspberry Pi OS list is long, because it includes image-handling libraries, but the heart of it is Python's virtual environment support, tkinter (for the turtle preview), the pigpio daemon and git:

```bash
sudo apt update
sudo apt install -y python3-venv python3-tk pigpiod git
```

The official page lists extra image libraries too (`libjpeg-dev` and friends), which you need for the image vectorising in lesson 6. Copy the full command from the install page if you want everything at once.

---

## Step 3: Make a virtual environment and get the library

A virtual environment keeps this project's Python packages separate from the rest of the system.

```bash
python3 -m venv env
source env/bin/activate
git clone https://github.com/evildmp/BrachioGraph.git
cd BrachioGraph
pip install -r requirements.txt
```

Your prompt shows `(env)` when the environment is active. Reactivate it with `source env/bin/activate` from the folder that contains `env` every time you open a new terminal. The requirements file installs NumPy, the pigpio Python module, Pillow, tqdm, readchar and pytest.

---

## Step 4: Start pigpiod and try an import

Each time you start up the plotter, the official docs run:

```bash
sudo pigpiod
source env/bin/activate
cd BrachioGraph
python
```

At the Python prompt `>>>`, check the library imports:

```python
from brachiograph import BrachioGraph
```

If no error appears, you are ready. Exit Python with `exit()`. We'll create the plotter itself in the next lesson, because creating it makes the servos move.

---

## Try it Yourself

1. Run `which python` with the environment active and without it. What changes?
2. Run `ls` inside the `BrachioGraph` folder and find `brachiograph.py` and `linedraw.py`. What do you think each does?
3. Challenge: stop and restart the pigpio service. The `sudo pigpiod` command starts it. How can you tell it is running (hint: look at the process list)?

---

## Common Issues

- **Problem**: `ModuleNotFoundError: No module named 'brachiograph'`.
- **Solution**: Run Python from inside the cloned `BrachioGraph` folder, with the virtual environment active.
- **Why**: The library is the `brachiograph.py` file in the repository, not a package installed from pip.

- **Problem**: `sudo apt install pigpiod` can't find the package.
- **Solution**: Check you are on the OS the official docs name, and check the pigpio project for your OS release.
- **Why**: Package names and availability change between OS releases, and we could not confirm them for every release.

- **Problem**: The servos don't move later and no error appears.
- **Solution**: Make sure `sudo pigpiod` is running.
- **Why**: If the library can't connect to the daemon it switches to virtual mode, so nothing moves.

- **Problem**: `pip install` fails with an "externally managed environment" message.
- **Solution**: Make sure the virtual environment is active (look for `(env)` in the prompt).
- **Why**: The system Python is protected on some OS releases, and a virtual environment is the safe place for packages.

---

## Check your understanding

{% quiz %}
? What does the pigpiod service do for the BrachioGraph?
- [ ] It converts images into line drawings
- [x] It generates the servo pulses on the GPIO pins
- [ ] It draws the on-screen turtle
> The library works out the angles, and pigpiod produces the pulses on the pins.

? Why do we use a virtual environment?
- [ ] To make the servos faster
- [x] To keep this project's Python packages separate from the system
- [ ] To connect to Wi-Fi
> The virtual environment holds the packages from requirements.txt without touching the system Python.

? What happens if the library can't reach pigpiod?
- [ ] The Pi reboots
- [x] It switches to a virtual plotter and nothing moves
- [ ] The servos move at half speed
> The official docs say it falls back to virtual mode, which is why a silent plotter is a good clue to check the daemon.
{% endquiz %}
