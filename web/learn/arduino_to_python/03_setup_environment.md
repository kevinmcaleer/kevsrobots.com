---
layout: lesson
title: Setting Up Your Python Development Environment
author: Kevin McAleer
type: page
cover: /learn/arduino_to_python/assets/setup_environment.jpg
date: 2025-01-20
previous: 02_arduino_uno_q.html
next: 04_variables_and_types.html
description: Installing tools and firmware for MicroPython development
percent: 12
duration: 11
navigation:
- name: Arduino to Python - A Developer's Guide
- content:
  - section: Getting Started
    content:
    - name: Introduction
      link: 00_intro.html
    - name: Why Python Alongside C++?
      link: 01_why_python.html
    - name: Understanding the Arduino Uno Q Architecture
      link: 02_arduino_uno_q.html
    - name: Setting Up Your Python Development Environment
      link: 03_setup_environment.html
  - section: Language Fundamentals
    content:
    - name: Variables and Data Types
      link: 04_variables_and_types.html
    - name: Control Flow - if, else, and Loops
      link: 05_control_flow.html
    - name: From setup() and loop() to Python Functions
      link: 06_functions.html
    - name: Arduino String Class vs Python str
      link: 07_strings.html
    - name: Fixed Arrays vs Dynamic Lists
      link: 08_arrays_vs_lists.html
  - section: Hardware Programming
    content:
    - name: Digital Pin Control
      link: 09_pin_control.html
    - name: Analog I/O - analogRead() and analogWrite() in MicroPython
      link: 10_analog_io.html
    - name: Serial Communication - Serial vs UART
      link: 11_serial_communication.html
    - name: Timing and Delays - delay(), millis(), and Non-Blocking Code
      link: 12_timing_and_delays.html
    - name: Interrupts and Event Handling
      link: 13_interrupts.html
  - section: Advanced Concepts
    content:
    - name: Memory Management - Manual vs Automatic
      link: 14_memory_management.html
    - name: Object-Oriented Programming
      link: 15_object_oriented.html
    - name: Arduino Libraries vs Python Modules
      link: 16_libraries_vs_modules.html
    - name: File Systems - A New Capability in Python
      link: 17_file_systems.html
  - section: Arduino Uno Q Dual Processor
    content:
    - name: Understanding the Dual-Processor System
      link: 18_understanding_dual_processor.html
    - name: MCU-MPU Communication Patterns
      link: 19_mpu_mcu_communication.html
    - name: Decision Matrix - When to Use What
      link: 20_when_to_use_what.html
  - section: Capstone Project
    content:
    - name: Capstone Project - Wall-Following Robot
      link: 21_project_overview.html
    - name: Building the C++ Version
      link: 22_building_c_version.html
    - name: Building the Python Version
      link: 23_building_python_version.html
    - name: Hybrid Approach - Best of Both Worlds
      link: 24_hybrid_approach.html
    - name: Course Summary and Next Steps
      link: 25_summary.html
---


![Setup Environment cover image](assets/setup_environment.jpg){:class="cover"}

## Introduction

You're already familiar with the Arduino IDE - click Upload, wait 30 seconds, and your code runs on the board. Python development is similar but with a few key differences.

In this lesson, you'll install everything you need to write MicroPython code for microcontrollers. We'll set up the tools, install MicroPython firmware on your board, and verify everything works with your first Python program.

By the end, you'll have a complete development environment and understand the workflow difference between Arduino C++ and MicroPython.

## What You'll Need

### Hardware
- **Raspberry Pi Pico**, **ESP32**, or **Arduino Uno Q WiFi**
- USB cable (micro-USB for Pico, USB-C or micro for ESP32)
- Computer (Windows, Mac, or Linux)

### Software (We'll Install These)
- **Python 3.x** - The language itself
- **Thonny IDE** - Beginner-friendly Python editor with MicroPython support
- **MicroPython firmware** - The Python interpreter for your microcontroller

## Arduino IDE vs Python IDE: Understanding the Difference

**Arduino IDE workflow:**
```
Write code → Compile → Upload → Run on board
       └─ Changes require re-compile and re-upload
```

**Python workflow:**
```
Write code → Save to board → Runs immediately
       └─ Can also test interactively in REPL
```

**Key differences:**
- **No compilation step** - Python is interpreted on the fly
- **Interactive REPL** - Test code without uploading a full program
- **Faster iteration** - Change a line, run it instantly

## Step 1: Install Python on Your Computer

MicroPython runs on your microcontroller, but you'll also want Python on your computer for testing and learning.

### Windows

1. Visit [python.org/downloads](https://python.org/downloads)
2. Download Python 3.11 or newer
3. Run the installer
4. **IMPORTANT:** Check "Add Python to PATH" during installation
5. Click "Install Now"

**Verify installation:**
- Open Command Prompt
- Type: `python --version`
- Should show: `Python 3.11.x` or similar

### macOS

Python 3 is usually pre-installed. Verify:

```bash
python3 --version
```

If not installed or outdated:
1. Install Homebrew: `/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"`
2. Install Python: `brew install python3`

### Linux (Debian/Ubuntu)

```bash
sudo apt update
sudo apt install python3 python3-pip
python3 --version
```

## Step 2: Install Thonny IDE

Thonny is perfect for Arduino developers learning Python. It's simple, has built-in MicroPython support, and includes a file manager for your board.

### All Platforms

1. Visit [thonny.org](https://thonny.org)
2. Download for your operating system
3. Install like any other application
4. Launch Thonny

**First launch configuration:**
1. Go to **Tools → Options**
2. Under "Interpreter" tab
3. Select "MicroPython (Raspberry Pi Pico)" or "MicroPython (ESP32)"
4. Click OK

We'll configure the specific board connection after installing firmware.

### Alternative: Visual Studio Code

If you prefer VS Code (more powerful but more complex):

1. Install [VS Code](https://code.visualstudio.com)
2. Install the "Pylance" extension
3. Install the "MicroPico" or "Pymakr" extension for MicroPython support

For this course, we'll use Thonny for its simplicity.

## Step 3: Install MicroPython Firmware on Your Board

This is analogous to the Arduino bootloader - it's the interpreter that runs your Python code. You only install it once.

### For Raspberry Pi Pico

**Download firmware:**
1. Visit [micropython.org/download/rp2-pico](https://micropython.org/download/rp2-pico)
2. Download the latest `.uf2` file (e.g., `rp2-pico-latest.uf2`)

**Install firmware:**
1. Hold the BOOTSEL button on your Pico
2. While holding, plug in the USB cable
3. Release BOOTSEL - Pico appears as a USB drive called "RPI-RP2"
4. Drag the `.uf2` file to the RPI-RP2 drive
5. The Pico will reboot automatically - **MicroPython is now installed!**

### For ESP32

**Download firmware:**
1. Visit [micropython.org/download/esp32](https://micropython.org/download/esp32)
2. Download the latest `.bin` file for your board variant

**Install esptool:**
```bash
pip3 install esptool
```

**Erase flash (first time only):**
```bash
esptool.py --port /dev/ttyUSB0 erase_flash
```

Note: Replace `/dev/ttyUSB0` with your port:
- **Windows:** `COM3`, `COM4`, etc.
- **macOS:** `/dev/tty.usbserial-*` or `/dev/cu.usbserial-*`
- **Linux:** `/dev/ttyUSB0` or `/dev/ttyACM0`

**Flash MicroPython:**
```bash
esptool.py --port /dev/ttyUSB0 --baud 460800 write_flash -z 0x1000 esp32-xxxxx.bin
```

**Reboot:**
- Unplug and replug your ESP32

### For Arduino Uno Q WiFi (QRB2210)

The Arduino Q's QRB2210 requires special attention as it's sharing the board with the STM32U585 processor.

**Note:** The Q's QRB2210 comes with WiFi firmware. Only flash MicroPython if you want to write custom Python code for the QRB2210 side.

1. Put QRB2210 in download mode (check Arduino documentation for Q-specific procedure)
2. Follow ESP32 flashing instructions above
3. Note: This may disable built-in WiFi support from the Arduino IDE

## Step 4: Connect Thonny to Your Board

Now let's verify everything works.

1. **Launch Thonny**
2. **Plug in your board** via USB
3. **Configure interpreter:**
   - Go to **Tools → Options → Interpreter**
   - Select **MicroPython (Raspberry Pi Pico)** or **MicroPython (ESP32)**
   - For Port/WebREPL, select your board's port (usually auto-detected)
   - Click **OK**

4. **Check connection:**
   - Look at the Shell panel at the bottom
   - You should see: `>>>` (the Python REPL prompt)
   - If you see it, you're connected!

**Troubleshooting:**
- **No >>> prompt:** Try unplugging and replugging the board
- **Wrong port:** Manually select the port in Options → Interpreter
- **Permission denied (Linux):** Run `sudo usermod -a -G dialout $USER`, then log out and back in
- **Still not working:** Press Stop/Restart button (red stop sign) in Thonny

## Step 5: Your First MicroPython Program

Let's verify everything works with the embedded systems "Hello World" - blinking an LED.

### In the REPL (Interactive Mode)

Type these lines directly in the Shell (bottom panel) and press Enter after each:

```python
>>> from machine import Pin
>>> led = Pin(25, Pin.OUT)
>>> led.on()
```

**On Raspberry Pi Pico:** The onboard LED should turn on!

**On ESP32:** Use GPIO2 instead: `led = Pin(2, Pin.OUT)`

Now turn it off:
```python
>>> led.off()
```

**Toggle it:**
```python
>>> led.toggle()
```

**This is the REPL** - the Read-Eval-Print Loop. You can test any Python command instantly without uploading a full program. This is incredibly powerful for debugging and experimentation.

### Writing a Full Program

Now let's write a proper blink program:

1. In the top panel (editor), type:

```python
from machine import Pin
import time

# Create LED object (Pin 25 on Pico, Pin 2 on ESP32)
led = Pin(25, Pin.OUT)

# Blink forever
while True:
    led.on()
    time.sleep(0.5)
    led.off()
    time.sleep(0.5)
```

2. **Save to your board:**
   - Go to **File → Save as...**
   - Choose **"MicroPython device"**
   - Name it `main.py`

3. **Run it:**
   - Press the green "Run" button (or F5)
   - Your LED should blink!

4. **Stop it:**
   - Press the red "Stop" button

**Important:** A file named `main.py` on your MicroPython board runs automatically when the board powers on (like Arduino's `setup()` and `loop()`).

## Comparing the Workflow

Let's see the difference between Arduino and MicroPython for the same task:

### Arduino C++ Workflow

```cpp
// Write code in Arduino IDE
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}
```

**Steps:**
1. Write code
2. Click Upload (30-60 second compile + upload)
3. Code runs
4. To change delay: modify code, re-upload (another 30-60 seconds)

### MicroPython Workflow

```python
from machine import Pin
import time

led = Pin(25, Pin.OUT)

while True:
    led.on()
    time.sleep(0.5)
    led.off()
    time.sleep(0.5)
```

**Steps:**
1. Write code
2. Press F5 (instant run)
3. To change delay: type `time.sleep(1.0)` in REPL and test immediately
4. Once satisfied, save the change

**Development time difference:** MicroPython's instant feedback can reduce iteration time from minutes to seconds.

## Understanding MicroPython File System

Your microcontroller now has a tiny file system!

**In Thonny:**
- Go to **View → Files**
- You'll see two panels:
  - **"This computer"** - Your PC's files
  - **"MicroPython device"** - Files on your board

You can drag files between them!

**Key files:**
- `main.py` - Runs automatically on boot (like Arduino sketch)
- `boot.py` - Runs before main.py (for system setup)
- Any other `.py` files - Can be imported as modules

This is completely different from Arduino:
- **Arduino:** One sketch uploaded, no file system
- **MicroPython:** Multiple files, can read/write data, save logs

## The REPL: Your New Best Friend

The REPL is like having a conversation with your microcontroller. This has no Arduino equivalent.

**Try these interactive explorations:**

```python
>>> 2 + 2
4

>>> name = "Robot"
>>> print(f"Hello from {name}")
Hello from Robot

>>> from machine import Pin
>>> pin = Pin(25, Pin.OUT)
>>> pin.value(1)  # Turn on
>>> pin.value(0)  # Turn off

>>> help(Pin)  # Get help on Pin class
# ... documentation appears ...
```

**Use the REPL for:**
- Testing hardware connections
- Debugging sensor readings
- Experimenting with new libraries
- Learning Python interactively
- Quick calculations

## Development Tools Comparison

| Feature | Arduino IDE | Thonny + MicroPython |
|---------|-------------|----------------------|
| Compilation | Yes (30-60s) | No (instant) |
| Interactive testing | No | Yes (REPL) |
| File system | No | Yes |
| Multiple files | Limited | Full support |
| On-device debugging | Limited | Interactive |
| Code completion | Basic | Good |
| Library management | Built-in | Manual/upip |

## Try It Yourself

### Exercise 1: REPL Exploration
Use the REPL to:
1. Calculate `255 / 3.3` (ADC voltage conversion)
2. Create a pin object for GPIO pin 15
3. Set it high, then low
4. Toggle it 5 times using a loop
5. Get help on the `time` module: `import time; help(time)`

### Exercise 2: Modify Blink Speed
Create three different `main.py` programs:
1. Blink at 1 Hz (0.5s on, 0.5s off)
2. Blink at 5 Hz (0.1s on, 0.1s off)
3. Blink with Morse code SOS pattern (... --- ...)

Time how long it takes to modify and test each version. Compare to Arduino workflow.

### Exercise 3: Multi-File Project
Create two files:
1. `config.py` with: `LED_PIN = 25` and `BLINK_DELAY = 0.5`
2. `main.py` that imports and uses these values

This shows how you can organize code across multiple files - harder in Arduino!

### Exercise 4: Interactive Pin Control
Use the REPL to control 3 LEDs on different pins simultaneously without writing a full program. Toggle them in different patterns.

## Common Setup Issues

### "Device not found" or "Port not available"
**Solution:**
- Check USB cable (some are power-only, need data cable)
- Try different USB port
- On Linux, check permissions: `ls -l /dev/ttyUSB0`
- Restart Thonny
- Unplug and replug board

### "Firmware upload failed"
**Solution:**
- For Pico: Make sure you held BOOTSEL while plugging in
- For ESP32: Try lower baud rate (115200 instead of 460800)
- Check you're using the correct firmware for your board variant

### "No module named 'machine'"
**Solution:**
- You're running regular Python, not MicroPython
- Check Thonny interpreter settings (Tools → Options → Interpreter)
- Make sure your board is selected, not "Local Python 3"

### "LED doesn't blink"
**Solution:**
- Wrong pin number - check your board's pinout
- Pico uses GPIO 25 for onboard LED
- ESP32 uses GPIO 2 for onboard LED
- Some boards don't have an onboard LED - add an external one

## Summary

You've now set up a complete MicroPython development environment:

**Installed:**
- Python 3.x on your computer
- Thonny IDE for code editing
- MicroPython firmware on your board

**Learned:**
- How to connect Thonny to your board
- Using the REPL for interactive testing
- Writing and running MicroPython programs
- Saving files to your board's file system

**Key Differences from Arduino:**
- No compilation step - instant feedback
- Interactive REPL for testing
- File system for multiple files and data
- Can modify code without full re-upload

**The workflow:**
```
Arduino: Edit → Compile → Upload → Test → Repeat
MicroPython: Edit → Run → Test → Edit (no upload!)
```

With your environment ready, you're now prepared to start translating your Arduino knowledge to Python. In the next lesson, we'll dive into variables and typing systems - how Python's dynamic typing compares to C++'s static typing.

---

> **Previous Lesson:** [Arduino Uno Q Architecture](/learn/arduino_to_python/02_arduino_uno_q_architecture.html) |
> **Next Lesson:** [Variables and Types](/learn/arduino_to_python/04_variables_and_types.html)
>
> **Course Home:** [Arduino to Python](/learn/arduino_to_python/)
