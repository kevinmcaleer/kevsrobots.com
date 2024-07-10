---
layout: lesson
title: MP Remote
author: Kevin McAleer
type: page
cover: assets/cover.png
date: 2024-07-07
previous: 13_mp_remote.html
description: MicroPython Remote - a command-line tool for managing MicroPython devices.
percent: 100
duration: 2
navigation:
- name: Intermediate level MicroPython
- content:
  - section: Introduction
    content:
    - name: Introduction to Intermediate MicroPython
      link: 01_intro.html
  - section: Object Oriented Programming
    content:
    - name: Object-Oriented Programming in MicroPython
      link: 02_oop.html
    - name: Classes and Objects
      link: 02a_classes.html
    - name: Abstraction
      link: 03_abstraction.html
    - name: Inheritance
      link: 04_inheritance.html
    - name: Encapsulation
      link: 05_encapsulation.html
    - name: Polymorphism
      link: 06_polymorphism.html
  - section: Decorators
    content:
    - name: Decorators
      link: 06a_decorators.html
  - section: Modules and Packages
    content:
    - name: Modules & Libraries
      link: 07_modules.html
    - name: Packages
      link: 08_packages.html
    - name: PyPi & MIP
      link: 09_pypi.html
  - section: DocStrings
    content:
    - name: DocStrings
      link: 10_docstrings.html
  - section: Interrupts and Timers
    content:
    - name: Interrupts
      link: 11_interrupts.html
    - name: Timers
      link: 12_timers.html
  - section: MP Remote
    content:
    - name: MP Remote
      link: 13_mp_remote.html
  - section: Conclusion
    content:
    - name: MP Remote
      link: 14_summary.html
---


## MP Remote

MP Remote is a command-line tool for managing MicroPython devices. It provides a simple and intuitive interface to interact with your MicroPython boards, upload files, run scripts, and more.

---

## Features

MP Remote offers the following features:

- **Interactive Shell**: Connect to your MicroPython device and run commands interactively.
- **File Management**: Upload, download, and delete files on your MicroPython device.
- **Script Execution**: Run Python scripts on your MicroPython device.
- **REPL Mode**: Enter the REPL mode to interact with your MicroPython device directly.
- **Device Information**: Get information about your MicroPython device, such as the firmware version and available memory.

---

## Installation

To install MP Remote, use the Python package manager `pip`. Run the following command in your terminal:

```bash
pip install mp-remote
```

---

## Usage

After installing MP Remote, use the `mp-remote` command in your terminal to interact with your MicroPython device. Here are some common commands:

### Connect to a Device

Connect to a MicroPython device using its serial port:

```bash
mp-remote connect /dev/ttyUSB0
```

---

### Run a Script

Execute a Python script on the connected MicroPython device:

```bash
mp-remote run script.py
```

---

### Upload a File

Upload a file to the connected MicroPython device:

```bash
mp-remote upload file.py
```

---

### Download a File

Download a file from the connected MicroPython device:

```bash
mp-remote download file.py
```

---

### Delete a File

Delete a file on the connected MicroPython device:

```bash
mp-remote delete file.py
```

---

### Enter REPL Mode

Enter the REPL mode to interact with the MicroPython device directly:

```bash
mp-remote repl
```

---

### Get Device Information

Get information about the connected MicroPython device:

```bash
mp-remote info
```

---

## Summary

MP Remote is a versatile tool for managing MicroPython devices. With its interactive shell, file management capabilities, and script execution features, you can easily work with your MicroPython boards and create exciting projects.

---
