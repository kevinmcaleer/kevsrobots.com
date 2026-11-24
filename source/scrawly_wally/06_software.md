---
title: Installing the Software
description: Install the GPIO library, check it works, and clone Kev's Scrawly Wally repository onto the Raspberry Pi.
layout: lesson
type: page
cover: /learn/scrawly_wally/assets/banners/06_software.jpg
date_updated: 2026-11-24
snakie: false
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to install the GPIO library on the Pi, check that Python can use it, and download Kev's Scrawly Wally code.

Time to give the Pi something to run. There are only two jobs: get a Python library that can switch the Pi's pins, and download Kev's project from GitHub. Run these commands on the Pi, over SSH or in a terminal on its desktop.

```mermaid
flowchart LR
    A["Install the GPIO library"] --> B["Check Python can import it"]
    B --> C["Install git"]
    C --> D["Clone the Scrawly Wally repository"]
```

## Step 1: the GPIO library

The programs in this course control the pins with `RPi.GPIO`, the library that lets Python set a pin high or low and make a PWM signal for the servo. Raspberry Pi OS provides it as a system package:

```sh
sudo apt-get install python3-rpi.gpio
```

If it's already there, apt will tell you so and carry on.

> **Note**: older guides tell you to use `pip3 install RPi.GPIO`. On recent versions of Raspberry Pi OS, pip refuses to install into the system Python and reports an `externally-managed-environment` error. The apt package avoids that.

## Step 2: check it works

Save this as `check_gpio.py` on the Pi and run it with `python3 check_gpio.py`:

```python
import RPi.GPIO as GPIO

print("RPi.GPIO version:", GPIO.VERSION)
print("Pi hardware info:", GPIO.RPI_INFO)
```

If it prints a version and some information about the Pi, you're ready. If you get `ModuleNotFoundError: No module named 'RPi'`, the library isn't installed for the Python you're running, so go back to step 1.

## Step 3: get Kev's code

The robot's software is on GitHub at <https://www.github.com/kevinmcaleer/scrawly-wally.git>. Install git if you need it, then clone the repository:

```sh
sudo apt-get install git
git clone https://www.github.com/kevinmcaleer/scrawly-wally.git
```

The repository is a work in progress and Kev updates it, so it may have moved on since this course was written. To fetch the latest changes, move into the folder and run:

```sh
cd scrawly-wally
git pull
```

**What about pigpio?** Kev's earlier notes mention the `pigpio` library, which makes steadier servo pulses and runs as a background daemon (`sudo pigpiod`). The programs in this course use only `RPi.GPIO`, so you don't need it to follow along. If you want to try it, check that it's available for your Pi and version of Raspberry Pi OS first, and read the repository for how it's used.

## Try it Yourself

1. Run the check program and write down your `RPi.GPIO` version.
2. Look inside the cloned `scrawly-wally` folder with `ls` and read the README. How is it organised?
3. Challenge: run `git log -3` inside the folder and find the date of the latest change.

## Common Issues

- **Problem**: `externally-managed-environment` when using pip.
- **Solution**: Use `sudo apt-get install python3-rpi.gpio` instead.
- **Why**: Recent Raspberry Pi OS stops pip changing the system Python, to protect the tools that depend on it.

- **Problem**: `git: command not found`.
- **Solution**: Run `sudo apt-get install git`.
- **Why**: Some Raspberry Pi OS images don't include git.

- **Problem**: `git clone` fails with a network error.
- **Solution**: Check the Pi is online and try again.
- **Why**: The Pi may not have joined WiFi yet, or the network may block GitHub.

> **Teachers**: GitHub can be blocked on school networks, and git clone downloads the whole project, so try it before the lesson. If it's blocked, download the repository on another computer and copy it across with a USB stick or `scp`. Every program in this course is complete in the lessons, so a blocked clone doesn't stop the course.

## Check your understanding

{% quiz %}
? What does the `RPi.GPIO` library do for us?
- [ ] It draws the pictures
- [x] It lets Python switch the Pi's pins and make the servo's PWM signal
- [ ] It connects the Pi to WiFi
> It's the library our programs use to control the pins.

? Why do we use apt, not pip, to install it?
- [ ] apt is faster
- [x] Recent Raspberry Pi OS blocks pip from changing the system Python
- [ ] pip does not know about the Pi
> The `externally-managed-environment` error is pip refusing to change the system Python.

? What does `git pull` do?
- [ ] Deletes the project
- [x] Fetches the latest changes into the folder you already cloned
- [ ] Uploads your changes to GitHub
> Pull brings down new commits from GitHub.
{% endquiz %}

Software installed. Now we make something move.
