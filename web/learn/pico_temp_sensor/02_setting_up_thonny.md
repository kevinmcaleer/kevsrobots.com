---
layout: lesson
title: Setting Up Thonny
author: Kevin McAleer
type: page
cover: /learn/pico_temp_sensor/assets/banners/02_setting_up_thonny.jpg
date: 2024-11-22
previous: 00_intro.html
next: 03_installing_micropython_on_pico.html
description: Get your environment ready by installing and configuring the Thonny IDE
  to program your Raspberry Pi Pico with MicroPython.
percent: 14
duration: 6
quiz_total: 36
date_updated: 2026-11-24
navigation:
- name: Using the Raspberry Pi Pico's Built-in Temperature Sensor
- content:
  - section: Introduction to the Raspberry Pi Pico and MicroPython
    content:
    - name: Course Introduction
      link: 00_intro.html
    - name: Setting Up Thonny
      link: 02_setting_up_thonny.html
    - name: Installing MicroPython on the Pico
      link: 03_installing_micropython_on_pico.html
  - section: Understanding the Built-in Temperature Sensor
    content:
    - name: "Understanding the Pico\u2019s Temperature Sensor"
      link: 04_overview_of_pico_temperature_sensor.html
    - name: Reading Temperature with the ADC
      link: 05_how_the_temperature_sensor_works.html
  - section: Writing Your First Temperature Sensor Script
    content:
    - name: Converting ADC Values to Temperature
      link: 06_reading_temperature_with_adc.html
    - name: Formatting Temperature Readings
      link: 07_displaying_temperature_in_thonny_console.html
    - name: Enhancing Temperature Readings
      link: 08_formatting_temperature_readings.html
  - section: Enhancing Your Temperature Monitoring Project
    content:
    - name: Getting More Accurate Temperature Readings
      link: 09_converting_adc_to_temperature.html
    - name: Creating a Temperature Alert System
      link: 10_creating_a_temperature_alert_system.html
  - section: Project Challenges and Next Steps
    content:
    - name: Project Ideas and Extensions
      link: 11_project_ideas_and_extensions.html
    - name: Troubleshooting Temperature Readings
      link: 12_troubleshooting_temperature_readings.html
    - name: Summary and Next Steps
      link: 13_summary_and_next_steps.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to install Thonny, connect your Pico, choose the right interpreter and run a program on the board.

Let's build your workbench. Before a Pico can tell you the temperature, you need somewhere to write code and a way to send it to the board. Thonny is a free, beginner-friendly editor that does both, and it already knows how to talk to a Pico.

```mermaid
flowchart LR
    A["Your computer: Thonny editor"] -- "USB cable" --> B["Pico running MicroPython"]
    B -- "print() output" --> C["Thonny Shell"]
```

When you press Run, Thonny sends your program over USB, the Pico runs it, and anything you `print()` comes back to the Shell at the bottom of the window.

---

## Step 1: Install Thonny

1. Visit [thonny.org](https://thonny.org) and download the latest version for your operating system.
2. Follow the installation instructions for your platform:
   - **Windows:** Run the installer and follow the prompts.
   - **macOS:** Open the downloaded `.pkg` file and follow the installer.
   - **Linux:** Use your package manager (for example `sudo apt install thonny` on Debian-based systems), or the install script from thonny.org for the newest version.
   - **Raspberry Pi OS:** Thonny is already installed.

---

## Step 2: Connect the Raspberry Pi Pico

1. Use a USB cable that carries data, not a charge-only cable. A charge-only cable powers the Pico but your computer will never see it.
2. **If your Pico already has MicroPython on it**, just plug it in. It shows up as a serial port, not as a drive.
3. **If your Pico is brand new (or you are not sure)**, hold down the **BOOTSEL** button while you plug in the USB cable, then let go. The Pico appears as a removable drive called `RPI-RP2` (or `RP2350` on a Pico 2). This bootloader mode is only needed to install firmware, which is the next lesson.

---

## Step 3: Choose the MicroPython interpreter

The interpreter is the Python that runs your code. Thonny can run your code on your computer or on the Pico, and you have to tell it which. This is the step beginners most often miss.

1. Open Thonny.
2. Go to **Tools > Options** and select the **Interpreter** tab (or click the interpreter name in the bottom-right corner of the Thonny window).
3. Choose **MicroPython (RP2040)** as the interpreter. Older versions of Thonny call it **MicroPython (Raspberry Pi Pico)**. Pick the Pico's port in the **Port** list, or leave it on **Try to detect port automatically**.
4. If your Pico is in BOOTSEL mode, click **Install or update MicroPython** in the same dialog, choose your board, and click **Install**. The next lesson shows how to do this by hand instead.

---

## Step 4: Check it works

Type this into Thonny's editor:

```python
print("Hello, Pico!")
```

Press the green **Run** button. Thonny asks where to save the file: choose **Raspberry Pi Pico**, and call it `hello.py`. You should see `Hello, Pico!` appear in the Shell. That message travelled to your Pico, ran there, and came back.

You're now ready to start programming your Pico.

> **Teachers**: installing Thonny normally needs permission to install software, and on a locked-down school computer you may need IT to do it, or to use the portable version from thonny.org. Learners sharing a computer should each save their files on their own Pico rather than on the computer. Label each Pico, because the files live on the board, not on the computer.

---

## Try it Yourself

1. Change the message to include your own name and run it again.
2. Add a second `print()` line with a calculation, such as `print(2 + 3)`, and predict the output before you run it.
3. Challenge: type `import machine` into the Shell at the bottom, press Enter, then type `help(machine)`. Scroll through the list. Can you spot `ADC` in it? That is the part we use in this course.

## Common Issues

- **Problem**: Thonny says it cannot find the Pico, or the port list is empty.
- **Solution**: Swap the USB cable for one you know carries data, plug straight into the computer rather than a hub, and select the interpreter again.
- **Why**: Many cables are power-only, and a port can only appear when the computer sees a USB data connection.

- **Problem**: The Shell shows a Python version for your computer, not a `MicroPython` banner.
- **Solution**: Open **Tools > Options > Interpreter** and choose **MicroPython (RP2040)**.
- **Why**: Thonny defaults to the Python on your computer, which has no idea your Pico exists.

- **Problem**: Run is greyed out or the red Stop button never goes away.
- **Solution**: Click Stop (or choose **Run > Stop/Restart backend**), or unplug and replug the Pico.
- **Why**: A program still running on the Pico can block new ones from starting.

## Check your understanding

{% quiz %}
? Which interpreter should you choose in Thonny for this course?
- [ ] Local Python 3
- [x] MicroPython (RP2040), or MicroPython (Raspberry Pi Pico) in older versions
- [ ] Any of them, they are the same
> The program has to run on the Pico, so Thonny must be set to a MicroPython interpreter and not your computer's own Python.

? Your computer cannot see the Pico at all. What is the most likely cause?
- [ ] The Pico has too many files on it
- [x] The USB cable is charge-only and carries no data
- [ ] Thonny is too old
> A charge-only cable powers the board but never gives the computer a data connection.

? When do you hold the BOOTSEL button?
- [ ] Every time you run a program
- [x] While plugging in the USB cable, when you want to install firmware
- [ ] To stop a running program
> BOOTSEL while powering up puts the Pico in bootloader mode, where it shows as a drive so you can install MicroPython.
{% endquiz %}
