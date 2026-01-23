---
layout: lesson
title: What is I2C?
author: Kevin McAleer
type: page
cover: /learn/i2c_spi/assets/cover.jpg
date: 2026-01-15
previous: 00_intro.html
next: 02_i2c-hardware-setup.html
description: Understand the I2C protocol, how it works, and why it's one of the most
  popular communication methods in electronics
percent: 14
duration: 7
navigation:
- name: Talking to the World - Working with I2C and SPI
- content:
  - section: Introduction
    content:
    - name: Introduction to I2C and SPI Communication
      link: 00_intro.html
  - section: Understanding I2C
    content:
    - name: What is I2C?
      link: 01_what-is-i2c.html
    - name: I2C Hardware Setup
      link: 02_i2c-hardware-setup.html
    - name: Scanning I2C Devices
      link: 03_scanning-i2c-devices.html
    - name: Reading I2C Sensors
      link: 04_reading-i2c-sensor.html
  - section: Working with Datasheets
    content:
    - name: Demystifying Datasheets
      link: 05_demystifying-datasheets.html
    - name: BME280 Deep Dive
      link: 06_bme280-deep-dive.html
  - section: Understanding SPI
    content:
    - name: What is SPI?
      link: 07_what-is-spi.html
    - name: SPI Hardware Setup
      link: 08_spi-hardware-setup.html
    - name: SPI Sensor Example
      link: 09_spi-sensor-example.html
  - section: Choosing and Troubleshooting
    content:
    - name: I2C vs SPI - Choosing the Right Protocol
      link: 10_i2c-vs-spi.html
    - name: Common Issues and Troubleshooting
      link: 11_common-issues.html
    - name: Final Project - Environmental Monitoring Station
      link: 12_final-project.html
---


<!-- ![I2C Communication Diagram](/learn/i2c_spi/assets/i2c-diagram.jpg){:class="img-fluid w-100"} -->

## Understanding I2C

I2C (Inter-Integrated Circuit, pronounced "I-squared-C" or "I-two-C") is a communication protocol that allows your Raspberry Pi Pico to talk to multiple devices using just two wires. It's incredibly popular in electronics because it's simple, reliable, and efficient.

Think of I2C like a conference call where everyone shares the same phone line. One person (the "controller" or "master") asks questions, and the others (the "peripherals" or "slaves") respond when called by their unique phone number (address).

## How I2C Works

### The Two-Wire System

I2C uses only two wires for all communication:

1. **SDA (Serial Data)** - Carries the actual data being sent back and forth
2. **SCL (Serial Clock)** - Keeps everyone synchronized, like a metronome

This is much simpler than other protocols that might need 4, 6, or even more wires. On the Raspberry Pi Pico, you'll typically use:

- **GPIO 8** for SDA
- **GPIO 9** for SCL

> **Note**: The Pico has two I2C buses (I2C0 and I2C1), giving you flexibility in pin selection. We'll use I2C0 in this course.

### Controller and Peripherals

In any I2C system, there's always:

- **One controller** (usually your Raspberry Pi Pico) - Initiates communication
- **One or more peripherals** (sensors, displays, etc.) - Respond when addressed

The controller is always in charge. It decides when to communicate and with which peripheral.

### Addressing System

Each I2C device on the bus has a unique 7-bit address (like a house number on a street). Common addresses look like:

- `0x77` - BME280 sensor (hexadecimal notation)
- `0x3C` - OLED display
- `0x68` - MPU6050 motion sensor

> **Important**: Two devices cannot share the same address on the same bus. Some devices let you change their address using jumpers or solder pads.

## Real-World I2C Example

Let's see I2C in action by initializing it on your Pico:

```python
from machine import I2C, Pin

# Create an I2C object on bus 0
# SDA on GP8, SCL on GP9, running at 400kHz
i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400000)

print("I2C initialized successfully")
print(f"I2C configuration: {i2c}")
```

**What this code does:**

- Imports the necessary classes from the `machine` module
- Creates an I2C object on bus 0 (I2C0)
- Configures SDA and SCL pins
- Sets communication speed to 400kHz (standard fast mode)

**Why this matters:**

This simple initialization unlocks communication with hundreds of different sensors and devices. Once I2C is set up, you can read temperature sensors, control LED matrices, communicate with real-time clocks, and much more - all through the same two wires.

## I2C Speed Modes

I2C supports different communication speeds:

| Mode | Speed | Use Case |
|------|-------|----------|
| Standard | 100 kHz | Basic sensors, simple devices |
| Fast | 400 kHz | Most common - good balance of speed and reliability |
| Fast Plus | 1 MHz | High-speed applications |
| High Speed | 3.4 MHz | Rare, specialized use |
{:class="table table-single table-narrow"}

For most projects with sensors like the BME280, use **Fast mode (400 kHz)**. It's reliable and supported by virtually all I2C devices.

```python
# Standard mode - slower but maximum compatibility
i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=100000)

# Fast mode - best for most projects
i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400000)
```

## Why Use I2C?

### Advantages

**Simple Wiring**
- Only two wires needed regardless of how many devices
- Easy to add more devices without rewiring
- Neat and organized projects

**Multiple Devices**
- Connect up to 127 devices on the same bus (theoretical limit)
- Each device has a unique address
- Controller can talk to any device at will

**Standardized**
- Industry-standard protocol
- Huge ecosystem of compatible devices
- Well-documented and supported

**Built-in Addressing**
- No need for separate chip select pins
- Software-controlled device selection
- Fewer GPIO pins consumed

### Limitations

**Speed**
- Slower than SPI (covered later in the course)
- Not suitable for very high-speed data transfer
- Fine for sensors but not for high-resolution displays

**Distance**
- Works best with short wires (under 1 meter)
- Signal quality degrades over distance
- Not suitable for long-distance communication

**Pull-up Resistors Required**
- Needs external resistors (we'll cover this in the next lesson)
- Can't work without them
- Sometimes causes confusion for beginners

## Real-World Applications

I2C is everywhere in electronics. Here are common devices you'll encounter:

**Sensors**
- BME280 - Temperature, humidity, pressure
- MPU6050 - Accelerometer and gyroscope
- BH1750 - Light sensor
- DS3231 - Real-time clock

**Displays**
- OLED displays (SSD1306)
- LCD displays with I2C backpacks
- 7-segment displays

**Expansion**
- Port expanders (PCF8574)
- ADC converters (ADS1115)
- PWM controllers (PCA9685)

## Common I2C Terms

You'll encounter these terms when working with I2C:

- **Bus** - The communication pathway (SDA and SCL wires)
- **Master/Slave** - Old terms for Controller/Peripheral
- **Pull-up resistor** - Required resistor on SDA and SCL lines
- **Address** - Unique identifier for each device (0x00 to 0x7F)
- **Scan** - Process of finding devices on the bus
- **Start/Stop condition** - Special signals to begin/end communication
- **ACK/NACK** - Acknowledge signals between devices

> **Note**: Modern documentation uses "Controller" and "Peripheral" instead of "Master" and "Slave", though you'll still see the old terminology in older datasheets.

## Try It Yourself

Let's experiment with I2C initialization:

1. **Try different speeds**: Change the `freq` parameter and see what your Pico accepts:

```python
from machine import I2C, Pin

# Try these different frequencies
for speed in [100000, 400000, 1000000]:
    i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=speed)
    print(f"I2C running at {speed} Hz")
```

2. **Use different pins**: The Pico is flexible with pin assignments. Try GPIO 4 and 5:

```python
# I2C on alternate pins
i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=400000)
print("I2C on alternate pins")
```

3. **Use the second I2C bus**: The Pico has two independent I2C buses:

```python
# I2C1 instead of I2C0
i2c1 = I2C(1, scl=Pin(15), sda=Pin(14), freq=400000)
print("Using second I2C bus")
```

## Common Mistakes

**Problem**: Code runs but no devices respond
**Solution**: Check that you've connected SDA and SCL to the correct pins
**Why**: I2C won't work if the pins don't match your code

**Problem**: "I2C bus error" message
**Solution**: Make sure you have pull-up resistors on SDA and SCL (covered in next lesson)
**Why**: I2C requires these resistors to function properly

**Problem**: Devices work individually but not together
**Solution**: Check that no two devices share the same address
**Why**: Address conflicts prevent proper communication

## What You've Learned

In this lesson, you discovered:

- I2C uses just two wires (SDA and SCL) to communicate with multiple devices
- The Raspberry Pi Pico acts as the controller, devices are peripherals
- Each device has a unique 7-bit address (like 0x77 or 0x3C)
- I2C runs at different speeds, with 400kHz being most common
- I2C is ideal for sensors and low-speed devices
- The protocol is standardized and widely supported

## What's Next

Now that you understand what I2C is, it's time to wire up a real sensor. In the next lesson, we'll connect a BME280 sensor to your Pico, covering proper wiring, pull-up resistors, and the physical connections that make I2C work.
