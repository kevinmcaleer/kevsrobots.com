---
title: Loading the code
description: Learn how to upload the burgerbot.py file to a Raspberry Pi Pico using Thonny IDE.
layout: lesson
type: page
---

## Uploading the code

To upload the `burgerbot.py` file to a Raspberry Pi Pico using [Thonny](https://thonny.org), follow these steps:

1. Connect the Raspberry Pi Pico to your computer using a USB cable.
1. Open Thonny on your computer.
1. Go to the **Tools** menu and select **Options**.
1. In the **Options** window, go to the **Interpreter** tab.
1. Select **MicroPython (Raspberry Pi Pico)** from the list.
1. Click **OK** to close the **Options** window.
1. Go to the **File** menu and select **Open**, then open `burgerbot.py` from your computer.
1. Go to the **File** menu and select **Save as**, choose **Raspberry Pi Pico**, and save it as `burgerbot.py`.
1. Create a new file, type `from burgerbot import Burgerbot` and run it with the green **Run** button. If there's no error, the library is installed.

> ## Make it run on its own
>
> Save a program on the Pico as `main.py` and it will run automatically whenever the robot is switched on, without a computer attached.

---
