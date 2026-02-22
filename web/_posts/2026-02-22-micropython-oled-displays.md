---
title: "From Code to Create Part 2: Adding OLED Displays with MicroPython"
description: >-
  Learn how to connect and control OLED displays with MicroPython. From basic text to graphics, animations, and real-time sensor data visualization.
excerpt: >-
  Master OLED displays in MicroPython! This episode covers I2C setup, text rendering, graphics, and building a real-time sensor dashboard.
layout: showcase
mode: light
date: 2026-02-22
author: Kevin McAleer
difficulty: intermediate
cover: /assets/img/blog/micropython_displays/cover.jpg
hero: /assets/img/blog/micropython_displays/hero.png
tags:
  - micropython
  - oled
  - i2c
  - displays
  - raspberry pi pico
  - ssd1306
groups:
  - micropython
  - electronics
  - maker
videos:
  - K0X4W09tiEA
stl:
 - name: MicroPython OLED Display stand
   link: /assets/stl/micropython_displays/oled_display_stand.stl
   description: The OLED display module for MicroPython projects
---

Ahoy there makers! Welcome to **Part 2** of our **From Code to Create** series! In this episode, we're diving into one of the most satisfying upgrades you can make to any MicroPython project: adding an **OLED display**.

Whether you're building a weather station, a robotics control panel, or just want to show off sensor readings in style, OLED screens are compact, crisp, and incredibly versatile. Best of all, they're surprisingly easy to work with in MicroPython.

In this guide, we'll cover everything you need to know about OLED displays:

- **What OLED displays are** and why they're perfect for maker projects
- **How to wire them up** to your Raspberry Pi Pico or any MicroPython board
- **Drawing text, shapes, and graphics** with the SSD1306 library
- **Building a real-time sensor dashboard** to visualize data
- **Animation and scrolling effects** to make your projects come alive

Let's get started!

---

## What Are OLED Displays?

**OLED** stands for **Organic Light-Emitting Diode**. Unlike traditional LCD screens that require a backlight, each pixel in an OLED display emits its own light. This gives you:

- **Incredible contrast** --- true blacks and vibrant whites
- **Wide viewing angles** --- readable from almost any direction
- **Low power consumption** --- pixels that are off don't use any power
- **Compact size** --- perfect for embedded projects

The most common OLED displays for maker projects use the **SSD1306** driver chip and come in two popular sizes:

- **0.96 inch** --- 128x64 pixels (what we'll use in this tutorial)
- **1.3 inch** --- 128x64 pixels (slightly larger, same resolution)

These displays typically communicate over **I2C** (using just two data pins plus power and ground), making them super easy to connect to any microcontroller.

---

## What You'll Need

Here's what you'll need to follow along:

### Hardware:
- A **Raspberry Pi Pico** (or any MicroPython-compatible board like ESP32, ESP8266, Pico W)
- A **0.96" OLED display** with SSD1306 driver (I2C version)
- **4 jumper wires** (female-to-female if using a breadboard)
- A **breadboard** (optional, for easier connections)
- A **USB cable** to connect your Pico to your computer

### Software:
- **Thonny IDE** (or your preferred MicroPython editor)
- **MicroPython** installed on your Raspberry Pi Pico

> **Tip:** Most OLED displays come with the SSD1306 driver already installed. Check the back of your display for the chip number to confirm. The I2C version will have 4 pins: VCC, GND, SCL, and SDA.

---

## Understanding I2C Communication

Before we connect the display, let's quickly talk about **I2C** (Inter-Integrated Circuit). It's a communication protocol that lets multiple devices share the same two data wires:

- **SCL (Serial Clock)** --- the clock signal that synchronizes data transfer
- **SDA (Serial Data)** --- the data line where information is sent

Each device on the I2C bus has a unique **address** (usually shown in hexadecimal like `0x3C`). This is how your microcontroller knows which device it's talking to.

The great thing about I2C is you can connect **multiple devices** to the same two pins. Want to add a temperature sensor and an OLED? No problem --- they can both share the same SCL and SDA lines.

---

## Wiring the OLED Display

Let's connect your OLED to the Raspberry Pi Pico. Here's the wiring:

| OLED Pin | Raspberry Pi Pico Pin |
|----------|------------------------|
| VCC      | 3.3V (pin 36)          |
| GND      | GND (pin 38)           |
| SCL      | GP1 (pin 2)            |
| SDA      | GP0 (pin 1)            |
{:class="table table-single table-narrow"}


---

> **Important:** Use the **3.3V pin**, not 5V. Most OLED displays are designed for 3.3V logic levels. Connecting to 5V might damage the display.

### Breadboard Setup (Optional)

If you're using a breadboard:

1. Insert the OLED display into the breadboard
2. Connect VCC to the Pico's 3.3V rail
3. Connect GND to the Pico's ground rail
4. Use jumper wires for SCL and SDA connections

Double-check your connections before powering on --- reversed power can damage the display.

---

## Finding Your Display's I2C Address

Once wired up, let's verify the display is connected and find its I2C address. Open Thonny and run this code:

```python
from machine import Pin, I2C

# Initialize I2C on pins GP0 (SDA) and GP1 (SCL)
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)

# Scan for I2C devices
devices = i2c.scan()

if devices:
    print("I2C devices found:")
    for device in devices:
        print(f"  - Address: 0x{device:02x}")
else:
    print("No I2C devices found. Check your wiring!")
```

You should see output like:

```
I2C devices found:
  - Address: 0x3c
```

The address `0x3c` (60 in decimal) is the default for most SSD1306 OLED displays. If your display uses a different address (like `0x3d`), just note it down --- you'll need it in the next step.

> **Troubleshooting:** If no devices are found, check your wiring. Make sure SCL and SDA aren't swapped, and verify your power connections.

---

## Installing the SSD1306 Library

MicroPython doesn't include the SSD1306 library by default, so we need to install it. There are two ways to do this:

### Method 1: Download Manually

1. Download the `ssd1306.py` library from the [Project GitHub repository](https://github.com/kevinmcaleer/oled_display/blob/main/ssd1306.py)
2. Save it to your Pico using Thonny (File → Save As → Raspberry Pi Pico)
3. Name it `ssd1306.py`

### Method 2: Copy the Code

Create a new file in Thonny called `ssd1306.py` and paste in this minimal version:

```python
from micropython import const
import framebuf

SET_CONTRAST = const(0x81)
SET_ENTIRE_ON = const(0xA4)
SET_NORM_INV = const(0xA6)
SET_DISP = const(0xAE)
SET_MEM_ADDR = const(0x20)
SET_COL_ADDR = const(0x21)
SET_PAGE_ADDR = const(0x22)
SET_DISP_START_LINE = const(0x40)
SET_SEG_REMAP = const(0xA0)
SET_MUX_RATIO = const(0xA8)
SET_COM_OUT_DIR = const(0xC0)
SET_DISP_OFFSET = const(0xD3)
SET_COM_PIN_CFG = const(0xDA)
SET_DISP_CLK_DIV = const(0xD5)
SET_PRECHARGE = const(0xD9)
SET_VCOM_DESEL = const(0xDB)
SET_CHARGE_PUMP = const(0x8D)

class SSD1306(framebuf.FrameBuffer):
    def __init__(self, width, height, external_vcc):
        self.width = width
        self.height = height
        self.external_vcc = external_vcc
        self.pages = self.height // 8
        self.buffer = bytearray(self.pages * self.width)
        super().__init__(self.buffer, self.width, self.height, framebuf.MONO_VLSB)
        self.init_display()

    def init_display(self):
        for cmd in (
            SET_DISP | 0x00,
            SET_MEM_ADDR, 0x00,
            SET_DISP_START_LINE | 0x00,
            SET_SEG_REMAP | 0x01,
            SET_MUX_RATIO, self.height - 1,
            SET_COM_OUT_DIR | 0x08,
            SET_DISP_OFFSET, 0x00,
            SET_COM_PIN_CFG, 0x02 if self.height == 32 else 0x12,
            SET_DISP_CLK_DIV, 0x80,
            SET_PRECHARGE, 0x22 if self.external_vcc else 0xF1,
            SET_VCOM_DESEL, 0x30,
            SET_CONTRAST, 0xFF,
            SET_ENTIRE_ON,
            SET_NORM_INV,
            SET_CHARGE_PUMP, 0x10 if self.external_vcc else 0x14,
            SET_DISP | 0x01):
            self.write_cmd(cmd)
        self.fill(0)
        self.show()

    def poweroff(self):
        self.write_cmd(SET_DISP | 0x00)

    def poweron(self):
        self.write_cmd(SET_DISP | 0x01)

    def contrast(self, contrast):
        self.write_cmd(SET_CONTRAST)
        self.write_cmd(contrast)

    def invert(self, invert):
        self.write_cmd(SET_NORM_INV | (invert & 1))

    def show(self):
        raise NotImplementedError

    def write_cmd(self, cmd):
        raise NotImplementedError

class SSD1306_I2C(SSD1306):
    def __init__(self, width, height, i2c, addr=0x3c, external_vcc=False):
        self.i2c = i2c
        self.addr = addr
        self.temp = bytearray(2)
        super().__init__(width, height, external_vcc)

    def write_cmd(self, cmd):
        self.temp[0] = 0x80
        self.temp[1] = cmd
        self.i2c.writeto(self.addr, self.temp)

    def write_data(self, buf):
        self.temp[0] = self.addr << 1
        self.temp[1] = 0x40
        self.i2c.start()
        self.i2c.write(self.temp)
        self.i2c.write(buf)
        self.i2c.stop()

    def show(self):
        x0 = 0
        x1 = self.width - 1
        if self.width == 64:
            x0 += 32
            x1 += 32
        self.write_cmd(SET_COL_ADDR)
        self.write_cmd(x0)
        self.write_cmd(x1)
        self.write_cmd(SET_PAGE_ADDR)
        self.write_cmd(0)
        self.write_cmd(self.pages - 1)
        self.i2c.writeto(self.addr, b'\x40' + self.buffer)
```

Save this file to your Pico as `ssd1306.py`.

---

## Your First Display: Hello World!

Let's test the display with the classic "Hello World" example:

```python
from machine import Pin, I2C
from ssd1306 import SSD1306_I2C

# Initialize I2C
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)

# Initialize display (128x64 pixels, I2C address 0x3c)
oled = SSD1306_I2C(128, 64, i2c)

# Clear the display
oled.fill(0)

# Write text
oled.text("Hello World!", 0, 0)
oled.text("OLED Display", 0, 20)
oled.text("with MicroPython", 0, 40)

# Update the display
oled.show()
```

Upload and run this code. Your OLED should light up with the text!

### How It Works:

1. **`oled.fill(0)`** --- Clears the display (0 = off/black, 1 = on/white)
2. **`oled.text("text", x, y)`** --- Draws text at position (x, y) in pixels
3. **`oled.show()`** --- Sends the buffer to the display to make changes visible

> **Remember:** You must call `oled.show()` after drawing to update the display. All drawing commands modify an internal buffer --- `show()` pushes that buffer to the screen.

---

## Drawing Shapes and Graphics

The SSD1306 library is built on MicroPython's `framebuf` module, which provides basic graphics functions:

### Drawing Pixels

```python
# Draw a single pixel at (x, y)
oled.pixel(64, 32, 1)  # 1 = on, 0 = off
oled.show()
```

### Drawing Lines

```python
# Draw a line from (x1, y1) to (x2, y2)
oled.line(0, 0, 127, 63, 1)
oled.show()
```

### Drawing Rectangles

```python
# Draw a rectangle at (x, y) with width and height
oled.rect(10, 10, 50, 30, 1)  # Outline only
oled.fill_rect(70, 10, 50, 30, 1)  # Filled rectangle
oled.show()
```

### Drawing Circles

```python
# Draw a circle at (x, y) with radius r
oled.ellipse(64, 32, 20, 20, 1)  # Circle (equal width and height)
oled.fill_ellipse(64, 32, 10, 10, 1)  # Filled circle
oled.show()
```

### Example: Smiley Face

```python
from machine import Pin, I2C
from ssd1306 import SSD1306_I2C

i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)
oled = SSD1306_I2C(128, 64, i2c)

oled.fill(0)

# Face outline
oled.ellipse(64, 32, 30, 30, 1)

# Eyes
oled.fill_ellipse(54, 25, 3, 3, 1)
oled.fill_ellipse(74, 25, 3, 3, 1)

# Smile
oled.ellipse(64, 32, 15, 15, 1)
oled.fill_rect(49, 20, 30, 15, 0)  # Erase top half

oled.show()
```

---

## Building a Real-Time Sensor Dashboard

Now let's build something practical: a dashboard that displays **temperature and humidity** from a DHT22 sensor in real time.

### Additional Hardware Needed:
- **DHT22 sensor** (temperature and humidity)
- **3 jumper wires**

### Wiring the DHT22:

| DHT22 Pin | Raspberry Pi Pico Pin |
|-----------|-----------------------|
| VCC       | 3.3V (pin 36)         |
| GND       | GND (pin 38)          |
| DATA      | GP15 (pin 20)         |

### The Code:

```python
from machine import Pin, I2C
from ssd1306 import SSD1306_I2C
import dht
import time

# Initialize I2C and OLED
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)
oled = SSD1306_I2C(128, 64, i2c)

# Initialize DHT22 sensor on GP15
sensor = dht.DHT22(Pin(15))

while True:
    try:
        # Read sensor data
        sensor.measure()
        temp = sensor.temperature()
        humidity = sensor.humidity()

        # Clear display
        oled.fill(0)

        # Draw header
        oled.text("Sensor Dashboard", 0, 0)
        oled.line(0, 10, 127, 10, 1)

        # Display temperature
        oled.text(f"Temp: {temp:.1f}C", 0, 20)

        # Display humidity
        oled.text(f"Humidity: {humidity:.1f}%", 0, 35)

        # Draw a simple bar graph for temperature (0-40C scale)
        bar_width = int((temp / 40) * 127)
        oled.rect(0, 50, 127, 10, 1)
        oled.fill_rect(0, 50, bar_width, 10, 1)

        # Update display
        oled.show()

        # Wait 2 seconds before next reading
        time.sleep(2)

    except OSError as e:
        print(f"Failed to read sensor: {e}")
        time.sleep(2)
```

This dashboard updates every 2 seconds with fresh sensor readings and includes a visual temperature bar graph!

---

## Adding Animation: Scrolling Text

Want to make your display more dynamic? Let's create scrolling text:

```python
from machine import Pin, I2C
from ssd1306 import SSD1306_I2C
import time

i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)
oled = SSD1306_I2C(128, 64, i2c)

message = "Hello from KevsRobots! OLED displays are awesome!"
x = 128  # Start off the right edge

while True:
    oled.fill(0)
    oled.text(message, x, 28)
    oled.show()

    x -= 2  # Move left by 2 pixels

    # Reset when text scrolls off the left edge
    if x < -len(message) * 8:  # 8 pixels per character
        x = 128

    time.sleep(0.05)  # Adjust speed here
```

---

## Try It Yourself

Here are some challenges to practice what you've learned:

### Beginner:
1. **Display your name** in large letters centered on the screen
2. **Create a simple menu** showing 3 options (use different y positions)
3. **Draw a house** using rectangles and lines

### Intermediate:
4. **Build a countdown timer** that displays seconds remaining
5. **Create a battery meter** showing a percentage bar
6. **Make a simple animation** (like a bouncing ball)

### Advanced:
7. **Build a clock** that displays hours, minutes, and seconds
8. **Create a game** (like Pong or Snake) using the display
9. **Display sensor data from multiple sensors** with labels and graphs

---

## Common Issues and Solutions

**Display stays blank:**
- Check your wiring (especially VCC and GND)
- Verify you're using 3.3V, not 5V
- Make sure you called `oled.show()` after drawing
- Check the I2C address matches your display

**Text is garbled or partially missing:**
- Your I2C frequency might be too high --- try `freq=200000` instead of `400000`
- Check for loose connections
- Make sure your text doesn't exceed the display boundaries (128x64)

**"OSError: [Errno 19] ENODEV" error:**
- The I2C device isn't responding
- Double-check your wiring
- Run the I2C scanner code to verify the address
- Some displays need a pull-up resistor on SCL/SDA lines

**Display works but is upside down:**
- You can rotate the display 180° in software by modifying the SSD1306 initialization (this is an advanced topic --- check the library documentation)

---

## What's Next?

Congratulations! You now know how to:

* ✅ Connect an OLED display to your Raspberry Pi Pico
* ✅ Display text, shapes, and graphics
* ✅ Build real-time sensor dashboards
* ✅ Create animations and scrolling effects

Happy making!

---

## Useful Links

- [MicroPython SSD1306 Driver Documentation](https://github.com/micropython/micropython/blob/master/drivers/display/ssd1306.py)
- [MicroPython FrameBuffer Documentation](https://docs.micropython.org/en/latest/library/framebuf.html)
- [Raspberry Pi Pico Pinout](https://www.raspberrypi.com/documentation/microcontrollers/raspberry-pi-pico.html)
- [I2C Protocol Explained](https://learn.sparkfun.com/tutorials/i2c)
- [DHT22 Sensor Documentation](https://docs.micropython.org/en/latest/esp8266/tutorial/dht.html)
- [Previous Episode: From Code to Create Part 1](https://www.kevsrobots.com/blog/from-code-to-creation.html)

---
