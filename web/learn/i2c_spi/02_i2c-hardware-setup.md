---
layout: lesson
title: I2C Hardware Setup
author: Kevin McAleer
type: page
cover: /learn/i2c_spi/assets/cover.jpg
date: 2026-01-15
previous: 01_what-is-i2c.html
next: 03_scanning-i2c-devices.html
description: Learn how to properly wire I2C devices with pull-up resistors and make
  reliable connections
percent: 21
duration: 9
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


![I2C Wiring Example](/learn/i2c_spi/assets/i2c-wiring.jpg){:class="img-fluid w-100"}

## Wiring Your First I2C Device

Now that you understand what I2C is, let's connect a real BME280 sensor to your Raspberry Pi Pico. This lesson covers everything you need to know about physical connections, power requirements, and the mysterious pull-up resistors.

## Understanding the BME280 Sensor

The BME280 is a fantastic sensor that measures:

- **Temperature** - Accurate to 1 degree Celsius
- **Humidity** - Percentage relative humidity
- **Barometric Pressure** - Atmospheric pressure

It communicates via I2C (or SPI), costs just a few dollars, and is perfect for weather stations, environmental monitoring, and learning I2C fundamentals.

> **Note**: BME280 modules come on small breakout boards that handle the complicated stuff (voltage regulation, filtering) so you can focus on learning.

## BME280 Pinout

Most BME280 breakout boards have 4 pins:

| Pin | Name | Description |
|-----|------|-------------|
| VCC | Power | 3.3V from Pico |
| GND | Ground | Common ground |
| SCL | Serial Clock | Connect to Pico GP9 |
| SDA | Serial Data | Connect to Pico GP8 |

> **Important**: Some BME280 boards can handle 5V power, but the Pico outputs 3.3V. Always use 3.3V to be safe.

## The Essential Pull-up Resistors

Here's something crucial about I2C: **the SDA and SCL lines need pull-up resistors**. Let me explain why.

### Why Pull-up Resistors Matter

I2C uses "open-drain" outputs. This means:

- Devices can pull the line LOW (to ground)
- Devices can release the line (let it float)
- **Devices cannot pull the line HIGH**

Without pull-up resistors, the lines would float randomly when released. The resistors pull them HIGH so devices only need to pull them LOW. Think of it like gravity - the resistor naturally lifts the line up, and devices pull it down when needed.

### Good News About Pull-ups

Most BME280 breakout boards already have pull-up resistors built in! Look at your board carefully:

- Small resistors labeled "103" (10kΩ) or "473" (47kΩ) near SDA/SCL
- Sometimes labeled "R1" and "R2"

If your board has these, you don't need to add external resistors. If it doesn't, you'll need to add 4.7kΩ or 10kΩ resistors between SDA/SCL and 3.3V.

```python
# You don't need to do anything in code for pull-ups
# They're purely hardware components
# Just know they must be present somewhere in the circuit
```

## Wiring the BME280 to Raspberry Pi Pico

Here's the complete wiring guide:

### Step-by-Step Connections

1. **Power Connection**
   - BME280 VCC → Pico 3.3V (pin 36)
   - BME280 GND → Pico GND (any GND pin)

2. **I2C Connection**
   - BME280 SCL → Pico GP9 (pin 12)
   - BME280 SDA → Pico GP8 (pin 11)

### Pico Pinout Reference

```
Raspberry Pi Pico Pinout (relevant pins):
┌─────────────────────────┐
│  USB PORT               │
├─────────────────────────┤
│ 1  GP0              VBUS│ 40
│ 2  GP1              VSYS│ 39
│ 3  GND               GND│ 38
│ 4  GP2           3V3_EN │ 37
│ 5  GP3              3.3V│ 36 ← Connect VCC here
│ 6  GP4               REF│ 35
│ 7  GP5               GP28│ 34
│ 8  GND               GND│ 33 ← Connect GND here
│ 9  GP6               GND│ 32
│10  GP7               GP27│ 31
│11  GP8 (SDA)         GP26│ 30 ← Connect SDA here
│12  GP9 (SCL)         RUN │ 29 ← Connect SCL here
└─────────────────────────┘
```

### Color Coding Tips

If you're using jumper wires, consider this color scheme:

- **Red** - Power (VCC to 3.3V)
- **Black** - Ground (GND to GND)
- **Yellow** - SCL (clock)
- **Blue** - SDA (data)

This makes troubleshooting easier and keeps your projects organized.

## Physical Connection Best Practices

### Use a Breadboard

For prototyping, breadboards are perfect:

```
Breadboard Layout:
┌────────────────────────────┐
│ + + + + + + + + + + + + +  │ ← Power rail (connect to 3.3V)
│ - - - - - - - - - - - - -  │ ← Ground rail (connect to GND)
│                            │
│  [BME280 Module]           │
│   VCC GND SCL SDA          │
│    │   │   │   │           │
│    └───┼───┼───┼──────┐    │
│        └───┼───┼──┐   │    │
│            └───┼┐ │   │    │
│                ││ │   │    │
└────────────────┼┼─┼───┼────┘
                 ││ │   │
              To Pico Pins
```

### Wire Length Matters

- Keep wires **short** - under 10cm if possible
- Longer wires increase capacitance and can cause communication errors
- If you need longer distances, consider I2C bus extenders

### Secure Connections

- Push wires firmly into breadboard holes
- Check that pins are making contact
- Wiggle wires gently - they shouldn't come loose

## Power Considerations

### Voltage Levels

The Raspberry Pi Pico uses 3.3V logic:

- **Safe**: 3.3V devices (like most BME280 boards)
- **Risky**: 5V-only devices (need level shifters)
- **Never**: Higher voltages - will damage your Pico

> **Warning**: Double-check your sensor's voltage requirements. Some sensors marked "5V" actually mean "can be powered by 5V" but use 3.3V logic. Read the datasheet to be sure.

### Power Supply Current

The BME280 uses very little current:

- **Typical**: 3.6μA (microamps) in sleep mode
- **Measuring**: 714μA during readings
- **Pico can supply**: Up to 300mA from 3.3V pin

This means you can power the BME280 directly from the Pico with no issues.

## Testing Your Wiring

Once everything is connected, let's verify the wiring with a simple test:

```python
from machine import I2C, Pin
import time

# Initialize I2C
i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400000)

print("Testing I2C connection...")
print("If this runs without errors, wiring is likely correct")

# Try to scan for devices (we'll learn more about this in the next lesson)
devices = i2c.scan()

if devices:
    print(f"Found {len(devices)} device(s)!")
    for device in devices:
        print(f"  - Device at address: 0x{device:02X}")
else:
    print("No devices found - check your wiring")
```

**What you should see:**

```
Testing I2C connection...
If this runs without errors, wiring is likely correct
Found 1 device(s)!
  - Device at address: 0x76
```

or

```
Found 1 device(s)!
  - Device at address: 0x77
```

> **Note**: BME280 sensors usually appear at address 0x76 or 0x77, depending on the manufacturer and whether they've connected the SDO pin to GND or VCC.

## Try It Yourself

Let's build confidence with your wiring:

1. **Wire your BME280** following the guide above
2. **Run the test code** and verify your device appears
3. **Intentionally disconnect SDA** - run the code again. You should see "No devices found"
4. **Reconnect SDA** - devices should reappear
5. **Try different GPIO pins** - modify the code to use GP4 (SDA) and GP5 (SCL), rewire accordingly

```python
# Using alternate pins
i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=400000)
devices = i2c.scan()
print(f"Devices on alternate pins: {devices}")
```

## Common Wiring Issues

### No Devices Found

**Problem**: Test code shows "No devices found"

**Possible causes**:
1. SDA or SCL wires not connected properly
2. Power not connected (VCC or GND loose)
3. Wrong pins in code vs physical wiring
4. Faulty sensor or breadboard connection

**Solution checklist**:
- [ ] Verify VCC connected to 3.3V pin
- [ ] Verify GND connected to GND pin
- [ ] Check SDA on GP8
- [ ] Check SCL on GP9
- [ ] Push all connections firmly
- [ ] Try different breadboard holes

### OSError or Bus Error

**Problem**: Error message like "OSError: [Errno 5] EIO" or "I2C bus error"

**Possible causes**:
1. Missing pull-up resistors
2. Short circuit between SDA and SCL
3. Sensor damaged

**Solutions**:
- Check that your BME280 board has pull-up resistors
- Ensure SDA and SCL wires aren't touching
- Try a different sensor if available

### Wrong Address

**Problem**: Device found but at unexpected address

**This is usually fine!** BME280 sensors can appear at either 0x76 or 0x77. Make note of your sensor's address - you'll need it for the next lesson.

```python
# If your sensor is at 0x76 instead of 0x77, just use that
BME280_ADDRESS = 0x76  # or 0x77
```

## Multiple I2C Devices

One of I2C's superpowers is connecting multiple devices to the same bus. Here's how:

### Wiring Multiple Devices

All devices share the same SDA and SCL lines:

```
Pico GP8 (SDA) ──┬── Device 1 SDA
                 ├── Device 2 SDA
                 └── Device 3 SDA

Pico GP9 (SCL) ──┬── Device 1 SCL
                 ├── Device 2 SCL
                 └── Device 3 SCL

Pico 3.3V ───────┬── Device 1 VCC
                 ├── Device 2 VCC
                 └── Device 3 VCC

Pico GND ────────┬── Device 1 GND
                 ├── Device 2 GND
                 └── Device 3 GND
```

Each device gets its own power and ground, but they share the communication lines.

> **Remember**: Each device needs a unique address. You can't have two BME280s at 0x76 on the same bus.

## What You've Learned

In this lesson, you mastered:

- BME280 pinout and pin functions (VCC, GND, SCL, SDA)
- Why I2C needs pull-up resistors and how to identify them
- Proper wiring from BME280 to Raspberry Pi Pico
- Best practices for breadboarding and wire management
- Power requirements and voltage levels
- Testing connections with scanning code
- Troubleshooting common wiring problems
- How to connect multiple I2C devices

Your BME280 is now physically connected and ready to communicate. You've verified it appears on the I2C bus with the test code.

## What's Next

Now that your sensor is wired and responding, let's learn how to scan the I2C bus systematically, understand device addresses, and why that "0x" notation keeps appearing. In the next lesson, we'll explore I2C scanning in detail and prepare to read actual sensor data.

---

> **Course Progress**: Lesson 2 of 12
>
> **Previous**: [What is I2C?](/learn/i2c_spi/01_what-is-i2c.html) |
> **Next**: [Scanning I2C Devices](/learn/i2c_spi/03_scanning-i2c-devices.html)
