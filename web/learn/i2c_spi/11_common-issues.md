---
layout: lesson
title: Common Issues and Troubleshooting
author: Kevin McAleer
type: page
cover: /learn/i2c_spi/assets/banners/11_common-issues.jpg
date: 2026-01-15
previous: 10_i2c-vs-spi.html
next: 12_final-project.html
description: Learn how to diagnose and fix common I2C and SPI problems
percent: 91
duration: 13
quiz_total: 36
date_updated: 2026-11-24
navigation:
- name: Talking to the World - Working with I2C and SPI
- content:
  - section: Introduction
    content:
    - name: Introduction to I2C and SPI Communication
      link: 00_intro.html
    - name: 'Video: Working with I2C'
      link: 13_course-video.html
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


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to work through a checklist to find the cause of a failing I2C or SPI device, and use MicroPython code to narrow it down.

## Systematic Troubleshooting

Hardware communication problems are frustrating but solvable. This lesson teaches you a systematic approach to diagnosing and fixing common I2C and SPI issues. With these techniques, you'll spend less time stuck and more time building.

## The Troubleshooting Mindset

Before diving into specific issues, adopt this approach:

1. **Start simple** - Verify power and ground first
2. **Isolate the problem** - Test one component at a time
3. **Use known-good components** - Swap parts to identify failures
4. **Check assumptions** - Is that pin really GP8?
5. **Read error messages carefully** - They often tell you exactly what's wrong

> **Golden Rule**: When stuck, go back to basics. Unplug everything and rebuild step-by-step.

## I2C Common Issues

### Issue 1: No Devices Found (Empty Scan)

**Symptom**:
```python
devices = i2c.scan()
print(devices)  # Prints: []
```
{: .no-snakie }

**Possible causes**:

1. **Missing pull-up resistors**

```python
# Check if your module has pull-up resistors
# Look for small resistors marked "103" (10kΩ), "472" (4.7kΩ) or "473" (47kΩ)
# If missing, add 4.7kΩ resistors:
# - SDA to 3.3V
# - SCL to 3.3V
```

2. **Wrong pins in code vs hardware**

```python
# Code says GP8/GP9 but wired to GP4/GP5
# Fix: Match code to actual wiring
i2c = I2C(0, scl=Pin(9), sda=Pin(8))  # Check physical connections!
```
{: .no-snakie }

3. **Power not connected**

```python
# Verify with multimeter:
# - VCC should read 3.3V
# - GND should read 0V
# If not, check power connections
```

4. **Device in wrong mode**

```python
# Some devices need a moment after power-up, or a reset pin pulled high,
# before they appear on the bus. Try power cycling:
# 1. Disconnect VCC for 5 seconds
# 2. Reconnect
# 3. Scan again
```

**Diagnostic script**:

```python
from machine import I2C, Pin
import time

def diagnose_i2c():
    """Comprehensive I2C diagnostics"""
    print("I2C Diagnostics Starting...\n")

    # Test 1: Initialize I2C
    print("Test 1: Initializing I2C...")
    try:
        i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=100000)  # Start slow
        print("[OK] I2C initialized at 100 kHz")
    except Exception as e:
        print(f"[FAIL] Failed to initialize I2C: {e}")
        return

    # Test 2: Scan for devices
    print("\nTest 2: Scanning for devices...")
    devices = i2c.scan()

    if devices:
        print(f"[OK] Found {len(devices)} device(s):")
        for addr in devices:
            print(f"  - 0x{addr:02X} (decimal {addr})")
    else:
        print("[FAIL] No devices found")
        print("\nTroubleshooting steps:")
        print("1. Check SDA and SCL connections")
        print("2. Verify VCC and GND connected")
        print("3. Confirm pull-up resistors present")
        print("4. Try power cycling the device")

    # Test 3: Try different speeds
    print("\nTest 3: Testing different speeds...")
    for speed in [100000, 400000]:
        i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=speed)
        devices = i2c.scan()
        status = "[OK]" if devices else "[FAIL]"
        print(f"{status} {speed/1000:.0f} kHz: {len(devices)} device(s)")

diagnose_i2c()
```

### Issue 2: OSError [Errno 5] EIO

**Symptom**:
```python
data = i2c.readfrom(0x76, 1)
# OSError: [Errno 5] EIO
```

**Meaning**: Input/Output Error - communication failed

**Causes**:

1. **Device not responding at that address**

```python
# Verify address with scan
devices = i2c.scan()
print(f"Available addresses: {[hex(d) for d in devices]}")

# Use correct address
if 0x76 in devices:
    data = i2c.readfrom(0x76, 1)
elif 0x77 in devices:
    data = i2c.readfrom(0x77, 1)
```

2. **Clock speed too fast**

```python
# Try slower speed
i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=100000)  # 100 kHz
```

3. **Loose connection**

```python
# Check all wires:
# - Push firmly into breadboard
# - Wiggle gently - should not come loose
# - Try different breadboard rows
```

### Issue 3: Wrong Data Returned

**Symptom**: Device responds but data is incorrect

**Causes**:

1. **Reading wrong register**

```python
# Double-check datasheet for register address
# BME280 chip ID is at 0xD0, not 0x00
chip_id = i2c.readfrom_mem(0x76, 0xD0, 1)  # Correct
# Not: i2c.readfrom_mem(0x76, 0x00, 1)
```

2. **Not waiting for measurement**

```python
# Some devices need time to complete measurement
i2c.writeto_mem(addr, REG_CTRL, b'\x01')  # Start measurement
time.sleep_ms(10)  # Wait for completion
data = i2c.readfrom_mem(addr, REG_DATA, 2)  # Now read
```

3. **Byte order reversed**

```python
# Some devices send MSB first, some LSB first
data = i2c.readfrom_mem(addr, reg, 2)

# MSB first (big-endian)
value = (data[0] << 8) | data[1]

# LSB first (little-endian)
value = (data[1] << 8) | data[0]
```

### Issue 4: Multiple Devices Interfere

**Symptom**: Works with one device but fails with multiple

**Causes**:

1. **Address conflict**

```python
# Check for duplicate addresses
devices = i2c.scan()
if len(devices) != len(set(devices)):
    print("WARNING: Duplicate addresses detected!")

# Note: scan() lists an address once even if two chips answer it, so a
# duplicate usually shows up as a device that behaves oddly or a scan
# that is missing a device you expected.
# Solution: change a device address if possible
# (e.g., BME280 SDO pin, OLED solder jumper)
```

2. **Too much capacitance (long wires)**

```python
# Symptom: Works with short wires, fails with long
# Solution: Use shorter wires (<20cm for multiple devices)
# Or: Lower I2C speed
i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=100000)
```

3. **Insufficient pull-up resistors**

```python
# Every breakout board's pull-ups are in parallel with the others, so
# three boards with 10kΩ each give about 3.3kΩ. That is usually fine.
# Problems come when the total drops below about 1kΩ (too many boards),
# or when a bare bus has none at all. Check the resistance with a meter
# (power off) between SDA and 3.3V.
```

## SPI Common Issues

### Issue 5: No Response from SPI Device

**Symptom**: Device doesn't respond to commands

**Causes**:

1. **Wrong SPI mode (polarity/phase)**

```python
# Try all 4 modes systematically (cs is the Pin you created for chip select)
for mode in range(4):
    polarity = mode >> 1
    phase = mode & 1
    spi = SPI(0, baudrate=1000000, polarity=polarity, phase=phase,
              sck=Pin(2), mosi=Pin(3), miso=Pin(4))

    cs.value(0)
    response = spi.read(1)
    cs.value(1)

    print(f"Mode {mode} (CPOL={polarity}, CPHA={phase}): {response}")
```
{: .no-snakie }

2. **Chip select not working**

```python
# Check CS logic level
# Most devices: LOW = selected, HIGH = deselected
cs = Pin(5, Pin.OUT)
cs.value(1)  # Start HIGH (deselected)

# Some devices: HIGH = selected (rare)
# Try inverted logic if standard doesn't work
```

3. **MOSI and MISO swapped**

```python
# Common mistake!
# MOSI (Controller Out) → Device SDI/MOSI/DIN
# MISO (Controller In) ← Device SDO/MISO/DOUT

# If swapped, reading returns zeros or 0xFF
# Solution: swap the jumper wires. (Hardware SPI0 only accepts GP3 as MOSI
# and GP4 as MISO in this wiring, so swapping them in code raises ValueError.)
```
{: .no-snakie }

### Issue 6: SPI Data Corrupted

**Symptom**: Some data correct, some garbage

**Causes**:

1. **Clock speed too fast**

<!-- code-check: skip -->
```python
# Start slow and increase gradually
spi = SPI(0, baudrate=100000, ...)   # 100 kHz
# Test, then increase:
spi = SPI(0, baudrate=1000000, ...)  # 1 MHz
spi = SPI(0, baudrate=10000000, ...) # 10 MHz
```
{: .no-snakie }

2. **Wires too long**

```python
# SPI is very sensitive to wire length
# Keep under 10cm for reliable operation
# Longer wires = signal reflections and noise
```

3. **Missing ground connection**

```python
# SPI needs solid ground reference
# Always connect:
# - Device GND to Pico GND
# - Use same GND point for all SPI devices
```

### Issue 7: Reading Wrong Number of Bytes

**Symptom**: Data format incorrect

**Cause**: Not sending dummy bytes for reading

```python
# WRONG: Only send the read command
tx_data = bytearray([0x80])  # Read register 0x00
rx_data = bytearray(1)
spi.write_readinto(tx_data, rx_data)
# Result: rx_data holds only what the device sent while it was still
# receiving the command (junk), not the register value!

# CORRECT: Send dummy bytes
tx_data = bytearray([0x80, 0x00])  # Command + dummy byte
rx_data = bytearray(2)
spi.write_readinto(tx_data, rx_data)
data = rx_data[1]  # Second byte has actual data
```

### Issue 8: Works on Breadboard, Fails on PCB

**Symptom**: Prototype works but final version doesn't

**Causes**:

1. **Signal integrity issues**

```python
# PCB traces too long or incorrect routing
# Solution: Keep SPI traces short (<5cm)
# Route SCLK away from other signals (crosstalk)
```

2. **Missing ground plane**

```python
# SPI needs good ground return path
# Solution: Use ground plane on PCB
# Or: Add ground traces alongside SPI signals
```

## Diagnostic Tools and Techniques

### Logic Analyzer (The Ultimate Tool)

If you have access to a logic analyzer:

```python
# Capture actual SPI/I2C signals
# See exactly what's being sent/received
# Identify:
# - Timing issues
# - Missing ACK (I2C)
# - Incorrect byte order
# - Wrong clock polarity/phase

# Low-cost option: Use another Pico as logic analyzer
# Software: Sigrok/PulseView (free)
```

### Multimeter Checks

```python
# Voltage checks:
# 1. VCC to GND: Should read 3.3V ±0.1V
# 2. SDA/SCL when idle: Should read 3.3V (pulled high)
# 3. CS when idle: Check expected level (usually HIGH)

# Continuity checks:
# 1. Verify each wire connection
# 2. Check for shorts between signals
# 3. Confirm ground connected
```

### Oscilloscope Analysis

```python
# What to check:
# 1. Clock signal: Clean square wave?
# 2. Data lines: Sharp transitions?
# 3. Rise/fall times: Sharp or rounded?
# 4. Noise: Any glitches or spikes?

# If signals look bad:
# - Shorten wires
# - Add decoupling capacitors (0.1µF near device VCC)
# - Lower clock speed
```

### Software Debugging

```python
def debug_i2c_read(i2c, addr, reg, num_bytes):
    """Debug I2C read with detailed output"""
    print(f"\nDebug: Reading from 0x{addr:02X}, reg 0x{reg:02X}")

    try:
        # Check device present
        devices = i2c.scan()
        if addr not in devices:
            print(f"[FAIL] Device 0x{addr:02X} not found on bus")
            print(f"  Available: {[hex(d) for d in devices]}")
            return None

        print(f"[OK] Device 0x{addr:02X} present")

        # Attempt read
        data = i2c.readfrom_mem(addr, reg, num_bytes)
        print(f"[OK] Read successful: {[hex(b) for b in data]}")
        return data

    except OSError as e:
        print(f"[FAIL] OSError: {e}")
        print("  Check: wiring, pull-ups, speed")
        return None

# Use it
data = debug_i2c_read(i2c, 0x76, 0xD0, 1)
```

## Preventive Measures

### Best Practices to Avoid Issues

**1. Always use error handling**

```python
try:
    temp = sensor.read_temperature()
    print(f"Temp: {temp}°C")
except OSError as e:
    print(f"Sensor error: {e}")
    # Fallback behavior
```
{: .no-snakie }

**2. Add timeouts**

```python
import time

def read_with_timeout(sensor, timeout_ms=1000):
    """Read with timeout protection"""
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
        try:
            return sensor.read()
        except:
            time.sleep_ms(10)
    return None
```

**3. Verify before use**

```python
class SafeSensor:
    def __init__(self, i2c, addr):
        self.i2c = i2c
        self.addr = addr
        self._verify_connection()

    def _verify_connection(self):
        """Verify sensor is connected"""
        if self.addr not in self.i2c.scan():
            raise RuntimeError(f"Sensor at 0x{self.addr:02X} not found")
        print(f"[OK] Sensor at 0x{self.addr:02X} verified")
```

**4. Add decoupling capacitors**

```python
# Hardware solution:
# Add 0.1µF ceramic capacitor between VCC and GND
# Place as close to sensor as possible
# Reduces power supply noise
```

## Emergency Recovery Techniques

### When All Else Fails

**1. Power cycle everything**

```python
# Disconnect Pico USB
# Wait 10 seconds
# Reconnect
# Some devices have strange startup states
```

**2. Try a different sensor/device**

```python
# If available, swap sensors
# Identifies if problem is:
# - Faulty sensor
# - Wrong wiring
# - Code issue
```

**3. Simplify to minimum**

```python
# Start fresh:
# 1. One device only
# 2. Shortest possible wires
# 3. Slowest speed
# 4. Simplest code

i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=100000)
print(i2c.scan())  # Just scan, nothing else
```

**4. Check for hidden connections**

```python
# Some breakout boards have:
# - Pull-up resistors you can't see
# - Address selection jumpers
# - Mode selection (I2C vs SPI)
# Read the board documentation carefully
```

## Try It Yourself

Practice troubleshooting:

1. **Intentionally create problems**:

<!-- code-check: skip -->
```python
# Wrong address
data = i2c.readfrom(0x99, 1)  # Doesn't exist

# Wrong pin
i2c = I2C(0, scl=Pin(99), sda=Pin(8))  # Invalid pin

# No CS control
spi.write(b'\x00')  # Forgot to use cs.value()
```

2. **Create a diagnostic tool**:

```python
def full_diagnostics():
    """Complete system check"""
    print("System Diagnostics\n" + "="*50)

    # Check I2C
    print("\nI2C Check:")
    try:
        i2c = I2C(0, scl=Pin(9), sda=Pin(8))
        devices = i2c.scan()
        print(f"  Found {len(devices)} I2C device(s): {[hex(d) for d in devices]}")
    except Exception as e:
        print(f"  I2C Error: {e}")

    # Check SPI
    print("\nSPI Check:")
    try:
        spi = SPI(0, sck=Pin(2), mosi=Pin(3), miso=Pin(4))
        print(f"  SPI initialized: {spi}")
    except Exception as e:
        print(f"  SPI Error: {e}")

    print("\n" + "="*50)

full_diagnostics()
```

## Common Issues at a Glance

- **Problem**: The I2C scan is empty
- **Solution**: Check power, SDA and SCL wiring, pull-up resistors and the pin numbers in code, in that order, then retry at 100 kHz
- **Why**: Nine times in ten a missing or loose wire looks identical to a dead device

- **Problem**: An SPI device returns all 0x00 or all 0xFF
- **Solution**: Check MISO and chip select wiring, then try the other SPI mode and a lower speed
- **Why**: A floating or mis-clocked MISO line reads as constant values

- **Problem**: It worked yesterday and fails today
- **Solution**: Unplug everything, rebuild the simplest circuit (one device, short wires) and add things back one at a time
- **Why**: Breadboard contacts loosen, and one change at a time shows which part broke

## Check your understanding

{% quiz %}
? What is the first thing to check when an I2C scan returns nothing?
- [ ] The Python version
- [x] Power, ground and the SDA and SCL wiring
- [ ] The chip ID
> Start with the simple, physical causes before debugging code.

? An SPI sensor returns 0xFF for every byte. Which is a likely cause?
- [ ] The scan is empty
- [x] MISO is unplugged, or the chip select is not being pulled low
- [ ] The pull-up resistors are missing
> SPI has no pull-ups or addresses, so check MISO, CS and the mode.

? Why does a long run of breadboard wires cause trouble at high speed?
- [ ] They use more power
- [x] They pick up noise and blur the fast clock edges
- [ ] The Pico cannot see them
> Signal quality falls as wires get longer, so shorten them or slow the clock.
{% endquiz %}

> **Teachers**: Troubleshooting is easiest to teach when the faults are on purpose. Have one learner secretly swap or unplug a wire on a working circuit, then let a partner find it with the checklist. A pair of spare, known-good sensors makes it quick to tell a faulty part from a faulty wire. A multimeter and a bag of jumper wires per bench is plenty; a logic analyser is nice to demonstrate but not needed.

## What You've Learned

In this lesson, you mastered:

- Systematic troubleshooting approaches
- Common I2C issues and solutions (no devices, OSError, wrong data)
- Common SPI issues and solutions (no response, corrupted data)
- Diagnostic techniques and tools
- Using multimeter and oscilloscope for debugging
- Software debugging strategies
- Preventive best practices
- Emergency recovery techniques

You can now diagnose and fix communication problems confidently, getting your projects working faster.

## What's Next

You've learned I2C, SPI, and troubleshooting. Now let's bring it all together! In the final lesson, we'll build a complete environmental monitoring station that combines I2C sensors, data logging, and everything you've learned in this course.
