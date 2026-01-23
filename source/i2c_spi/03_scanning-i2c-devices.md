---
title: Scanning I2C Devices
description: Learn how to scan for I2C devices, understand hexadecimal addresses, and identify what's connected to your bus
layout: lesson
type: page
---

![I2C Scanner Output](/learn/i2c_spi/assets/i2c-scanner.jpg){:class="img-fluid w-100"}

## Finding Devices on the I2C Bus

One of the most useful tools in your I2C toolkit is the ability to scan the bus and discover what devices are connected. This lesson teaches you how scanning works, what those hexadecimal addresses mean, and how to use scanning to troubleshoot projects.

## Why Scan for Devices?

Scanning the I2C bus helps you:

- **Verify connections** - Confirm a device is wired correctly
- **Find addresses** - Determine what address a device is using
- **Troubleshoot** - Identify missing or conflicting devices
- **Debug projects** - See exactly what's connected before writing code
- **Learn** - Understand which devices are present on breakout boards

> **Think of it like**: Network scanning on a computer network, but for electronics. You're asking "who's out there?" and devices respond with their address.

## How I2C Scanning Works

The scanning process is simple:

1. Controller (your Pico) tries to communicate with every possible address (0x00 to 0x7F)
2. If a device responds at that address, it's detected
3. If no response, that address is empty
4. Results show which addresses have active devices

MicroPython makes this incredibly easy with the `scan()` method:

```python
from machine import I2C, Pin

# Initialize I2C
i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400000)

# Scan for devices
devices = i2c.scan()

if devices:
    print(f"Found {len(devices)} device(s):")
    for device in devices:
        print(f"  - Decimal address: {device}")
        print(f"  - Hex address: 0x{device:02X}")
else:
    print("No I2C devices found")
```

**Example output with BME280 connected:**

```
Found 1 device(s):
  - Decimal address: 118
  - Hex address: 0x76
```

## Understanding Hexadecimal Addresses

You'll notice addresses appear in two formats:

- **Decimal**: 118
- **Hexadecimal**: 0x76

### Why Hexadecimal?

Hexadecimal (base-16) is used in electronics because:

- It's more compact than binary
- It's easier to read than long binary strings
- Each hex digit represents exactly 4 bits
- It's the standard in datasheets and documentation

### Reading Hex Addresses

The "0x" prefix means "this number is in hexadecimal":

| Decimal | Hexadecimal | Binary |
|---------|-------------|--------|
| 0 | 0x00 | 0000 0000 |
| 16 | 0x10 | 0001 0000 |
| 118 | 0x76 | 0111 0110 |
| 119 | 0x77 | 0111 0111 |
| 127 | 0x7F | 0111 1111 |

> **Note**: I2C uses 7-bit addresses, so the maximum is 0x7F (127 in decimal). There are technically 128 possible addresses, but some are reserved.

### Common Device Addresses

Here are addresses you'll frequently encounter:

| Device | Address | Hex |
|--------|---------|-----|
| BME280 | 118 or 119 | 0x76 or 0x77 |
| SSD1306 OLED | 60 or 61 | 0x3C or 0x3D |
| MPU6050 | 104 or 105 | 0x68 or 0x69 |
| DS3231 RTC | 104 | 0x68 |
| PCF8574 Port Expander | 32-39 | 0x20-0x27 |
| ADS1115 ADC | 72-75 | 0x48-0x4B |

## Building a Better Scanner

Let's create a more informative I2C scanner that shows the full address range:

```python
from machine import I2C, Pin

def scan_i2c():
    """Scan I2C bus and display results in a table"""
    i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400000)

    print("\nScanning I2C bus...")
    print("=" * 40)

    devices = i2c.scan()

    if not devices:
        print("No devices found!")
        return

    print(f"Found {len(devices)} device(s):\n")

    print("Address  | Decimal | Binary")
    print("-" * 40)

    for device in devices:
        binary = format(device, '08b')
        print(f"0x{device:02X}    | {device:3d}     | {binary}")

    print("=" * 40)

# Run the scanner
scan_i2c()
```

**Example output:**

```
Scanning I2C bus...
========================================
Found 2 device(s):

Address  | Decimal | Binary
----------------------------------------
0x3C    |  60     | 00111100
0x76    | 118     | 01110110
========================================
```

This shows you have an OLED display (0x3C) and a BME280 sensor (0x76) connected.

## Reserved Addresses

Not all addresses can be used by devices. Some are reserved by the I2C specification:

| Address Range | Hex | Purpose |
|---------------|-----|---------|
| 0x00-0x07 | 0x00-0x07 | Reserved (bus control) |
| 0x78-0x7F | 0x78-0x7F | Reserved (10-bit addressing) |

Regular devices use addresses from **0x08 to 0x77** (8 to 119 decimal).

```python
# Check if an address is valid for devices
def is_valid_device_address(addr):
    """Check if address is in valid device range"""
    return 0x08 <= addr <= 0x77

address = 0x76
if is_valid_device_address(address):
    print(f"0x{address:02X} is a valid device address")
```

## Interactive Scanner with Device Recognition

Let's build a scanner that recognizes common devices:

```python
from machine import I2C, Pin

# Database of common I2C devices
KNOWN_DEVICES = {
    0x3C: "SSD1306 OLED Display (128x64)",
    0x3D: "SSD1306 OLED Display (128x32)",
    0x68: "MPU6050 IMU / DS3231 RTC",
    0x69: "MPU6050 IMU (ALT address)",
    0x76: "BME280 / BMP280 Sensor",
    0x77: "BME280 / BMP280 Sensor (ALT)",
    0x48: "ADS1115 ADC",
    0x20: "PCF8574 Port Expander",
}

def smart_scan():
    """Scan and identify devices"""
    i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400000)

    print("\n🔍 Smart I2C Scanner")
    print("=" * 50)

    devices = i2c.scan()

    if not devices:
        print("❌ No devices found")
        print("\nTroubleshooting tips:")
        print("  - Check SDA and SCL connections")
        print("  - Verify power (VCC and GND)")
        print("  - Ensure pull-up resistors present")
        return

    print(f"✅ Found {len(devices)} device(s):\n")

    for device in devices:
        hex_addr = f"0x{device:02X}"
        device_name = KNOWN_DEVICES.get(device, "Unknown device")

        print(f"📍 Address: {hex_addr} ({device})")
        print(f"   Likely: {device_name}")
        print()

    print("=" * 50)

# Run the smart scanner
smart_scan()
```

**Example output:**

```
🔍 Smart I2C Scanner
==================================================
✅ Found 1 device(s):

📍 Address: 0x76 (118)
   Likely: BME280 / BMP280 Sensor

==================================================
```

## Try It Yourself

Experiment with I2C scanning:

1. **Scan with just the BME280** - Note its address

```python
from machine import I2C, Pin

i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400000)
devices = i2c.scan()
print(f"BME280 address: 0x{devices[0]:02X}")
```

2. **Temporarily disconnect SDA** - Run the scan. You should see no devices

3. **Reconnect and scan again** - Device should reappear

4. **Scan at different speeds** - Does it affect detection?

```python
# Try slower speed
i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=100000)
print(f"100kHz scan: {i2c.scan()}")

# Try faster speed
i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400000)
print(f"400kHz scan: {i2c.scan()}")
```

5. **Challenge**: Modify the smart scanner to check EVERY address (0x00 to 0x7F) and show which are empty vs occupied:

```python
from machine import I2C, Pin

i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400000)

print("Full address scan (0x00 to 0x7F):")
for addr in range(0x00, 0x80):
    if addr in i2c.scan():
        print(f"0x{addr:02X}: DEVICE FOUND")
    else:
        print(f"0x{addr:02X}: empty")
```

## Address Conflicts

What happens if two devices have the same address? **They conflict and communication fails.**

### Avoiding Conflicts

Many devices let you change their address:

**Hardware jumpers** - Solder pads or physical jumpers
- BME280: SDO pin changes 0x76 ↔ 0x77
- SSD1306: Resistor changes 0x3C ↔ 0x3D

**Software configuration** - Some devices allow address changes via commands
- PCF8574: Has 8 different possible addresses via A0, A1, A2 pins

### Detecting Conflicts

If your scanner shows no devices but you know hardware is connected:

```python
from machine import I2C, Pin

i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400000)

# Try to read from suspected address
address = 0x76

try:
    i2c.readfrom(address, 1)
    print(f"Device at 0x{address:02X} is responding")
except OSError:
    print(f"No device or error at 0x{address:02X}")
```

## Practical Scanning Applications

### 1. Pre-Flight Check

Before running your main program, verify devices are present:

```python
from machine import I2C, Pin

def check_devices(expected_addresses):
    """Verify expected devices are present"""
    i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400000)
    found = i2c.scan()

    for addr in expected_addresses:
        if addr in found:
            print(f"✓ Found device at 0x{addr:02X}")
        else:
            print(f"✗ Missing device at 0x{addr:02X}")
            return False

    return True

# Check for BME280 before running main code
if check_devices([0x76]):
    print("All devices present - starting main program")
else:
    print("ERROR: Check your wiring!")
```

### 2. Auto-Detection

Let your code adapt to what's actually connected:

```python
from machine import I2C, Pin

i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400000)
devices = i2c.scan()

# Check which BME280 address is present
if 0x76 in devices:
    BME280_ADDR = 0x76
    print("Using BME280 at 0x76")
elif 0x77 in devices:
    BME280_ADDR = 0x77
    print("Using BME280 at 0x77")
else:
    print("No BME280 found!")
    BME280_ADDR = None
```

## Common Issues

### Scanner Shows No Devices

**Problem**: `i2c.scan()` returns empty list `[]`

**Solutions**:
- Verify SDA and SCL are connected
- Check power connections (VCC and GND)
- Ensure pull-up resistors are present
- Try lower speed (100kHz instead of 400kHz)
- Test with known-good sensor

### Scanner Shows Unexpected Address

**Problem**: Device appears at wrong address

**This is normal if**:
- Different manufacturer uses alternate address
- Hardware jumper is in different position
- Device has configurable address

**Solution**: Note the actual address and use it in your code

### Scanner Shows Multiple Unknown Addresses

**Problem**: Unexpected devices appear

**Possible causes**:
- Breadboard has other devices connected
- I2C bus extender or multiplexer present
- Hidden I2C components on breakout board

**Solution**: Disconnect devices one at a time to identify each address

## What You've Learned

In this lesson, you mastered:

- Using `i2c.scan()` to detect devices on the bus
- Understanding hexadecimal address notation (0x76, etc.)
- Converting between decimal and hex addresses
- Building informative scanners with device recognition
- Identifying common I2C device addresses
- Detecting and avoiding address conflicts
- Using scanning for troubleshooting and verification
- Creating pre-flight checks for your projects

You now have powerful diagnostic tools to work with any I2C device. Scanning is your first step in every I2C project - it confirms hardware is working before you write complex code.

## What's Next

Your BME280 is connected, verified, and you know its address. Now it's time to actually read data from it! In the next lesson, we'll communicate with the sensor, read temperature, humidity, and pressure values, and display them in real-time.

---

> **Course Progress**: Lesson 3 of 12
>
> **Previous**: [I2C Hardware Setup](/learn/i2c_spi/02_i2c-hardware-setup.html) |
> **Next**: [Reading I2C Sensors](/learn/i2c_spi/04_reading-i2c-sensor.html)
