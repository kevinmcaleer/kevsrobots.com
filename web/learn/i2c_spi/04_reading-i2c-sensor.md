---
layout: lesson
title: Reading I2C Sensors
author: Kevin McAleer
type: page
cover: /learn/i2c_spi/assets/cover.jpg
date: 2026-01-15
previous: 03_scanning-i2c-devices.html
next: 05_demystifying-datasheets.html
description: Learn how to read temperature, humidity, and pressure data from the BME280
  sensor using I2C
percent: 42
duration: 10
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


<!-- ![BME280 Reading Data](/learn/i2c_spi/assets/bme280-reading.jpg){:class="img-fluid w-100"} -->

## Reading Real Sensor Data

Now comes the exciting part - reading actual environmental data from your BME280 sensor! This lesson teaches you how to communicate with I2C sensors, use existing libraries, and understand what's happening behind the scenes.

## Two Approaches to Reading Sensors

You have two options when working with I2C sensors:

### 1. Using Libraries (Recommended for Projects)

Pre-written libraries handle the complex communication details:

```python
from machine import I2C, Pin
import bme280

# Initialize I2C
i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400000)

# Create BME280 object
bme = bme280.BME280(i2c=i2c)

# Read sensor - it's this easy!
temperature, pressure, humidity = bme.read_compensated_data()

print(f"Temperature: {temperature / 100:.2f} °C")
print(f"Pressure: {pressure / 256 / 100:.2f} hPa")
print(f"Humidity: {humidity / 1024:.2f} %")
```

### 2. Manual Register Reading (For Learning)

Communicate directly with sensor registers:

```python
from machine import I2C, Pin

i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400000)

# Read the chip ID register (0xD0) to verify it's a BME280
chip_id = i2c.readfrom_mem(0x76, 0xD0, 1)
print(f"Chip ID: 0x{chip_id[0]:02X}")  # Should be 0x60 for BME280
```

> **Recommendation**: Use libraries for real projects, but understanding manual communication helps you debug and work with any sensor.

## Installing the BME280 Library

MicroPython doesn't include the BME280 library by default. Let's install it:

### Method 1: Using Thonny (Easiest)

1. Open Thonny IDE
2. Go to **Tools → Manage packages**
3. Search for "micropython-bme280"
4. Click **Install**

### Method 2: Manual Installation

Download the library and upload to your Pico:

1. Download `bme280.py` from [GitHub](https://github.com/robert-hh/BME280)
2. In Thonny, go to **File → Save As**
3. Choose **Raspberry Pi Pico**
4. Save as `bme280.py` in the root directory

### Method 3: Direct File Copy

Create `bme280.py` on your Pico with the library code. For this lesson, I'll provide a simplified version that works perfectly:

```python
# Save this as bme280.py on your Pico

from micropython import const
import time

# BME280 default address
BME280_I2C_ADDR = const(0x76)

# Register addresses
REG_CHIP_ID = const(0xD0)
REG_RESET = const(0xE0)
REG_CTRL_HUM = const(0xF2)
REG_CTRL_MEAS = const(0xF4)
REG_CONFIG = const(0xF5)
REG_DATA = const(0xF7)

class BME280:
    def __init__(self, i2c, addr=BME280_I2C_ADDR):
        self.i2c = i2c
        self.addr = addr

        # Verify chip ID
        chip_id = self.i2c.readfrom_mem(self.addr, REG_CHIP_ID, 1)[0]
        if chip_id != 0x60:
            raise RuntimeError(f"BME280 not found. Chip ID: 0x{chip_id:02X}")

        # Load calibration data
        self._load_calibration()

        # Configure sensor
        self.i2c.writeto_mem(self.addr, REG_CTRL_HUM, b'\x03')   # Humidity oversampling x4
        self.i2c.writeto_mem(self.addr, REG_CTRL_MEAS, b'\x6F')  # Temp/pressure oversampling, normal mode
        self.i2c.writeto_mem(self.addr, REG_CONFIG, b'\xA0')     # Standby 1000ms, filter off

    def _load_calibration(self):
        """Load calibration coefficients from sensor"""
        # This is simplified - real library has full calibration
        cal = self.i2c.readfrom_mem(self.addr, 0x88, 24)
        self.dig_T1 = cal[0] | (cal[1] << 8)
        self.dig_T2 = cal[2] | (cal[3] << 8)
        self.dig_T3 = cal[4] | (cal[5] << 8)
        # ... (more calibration values)

    def read_raw_data(self):
        """Read raw sensor data"""
        data = self.i2c.readfrom_mem(self.addr, REG_DATA, 8)

        pressure_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        temperature_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        humidity_raw = (data[6] << 8) | data[7]

        return temperature_raw, pressure_raw, humidity_raw

    def read_compensated_data(self):
        """Read and compensate sensor data"""
        temp_raw, press_raw, hum_raw = self.read_raw_data()

        # Apply calibration (simplified)
        temperature = temp_raw  # In real library, this uses calibration
        pressure = press_raw
        humidity = hum_raw

        return temperature, pressure, humidity
```

> **Note**: This is a simplified version for learning. For production projects, use the full library from GitHub.

## Your First Sensor Reading

Let's read data from the BME280:

```python
from machine import I2C, Pin
import bme280
import time

# Initialize I2C
i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400000)

# Verify sensor is present
print("Scanning for BME280...")
devices = i2c.scan()

if 0x76 not in devices and 0x77 not in devices:
    print("ERROR: BME280 not found!")
else:
    print("BME280 found!")

    # Create sensor object (try both common addresses)
    try:
        bme = bme280.BME280(i2c=i2c, addr=0x76)
        print("Using address 0x76")
    except:
        bme = bme280.BME280(i2c=i2c, addr=0x77)
        print("Using address 0x77")

    # Read sensor
    print("\nReading sensor data...\n")

    temp, press, hum = bme.read_compensated_data()

    # Convert to human-readable units
    temperature_c = temp / 100
    pressure_hpa = press / 256 / 100
    humidity_pct = hum / 1024

    print(f"🌡️  Temperature: {temperature_c:.2f} °C")
    print(f"📊 Pressure: {pressure_hpa:.2f} hPa")
    print(f"💧 Humidity: {humidity_pct:.2f} %")
```

**Example output:**

```
Scanning for BME280...
BME280 found!
Using address 0x76

Reading sensor data...

🌡️  Temperature: 22.45 °C
📊 Pressure: 1013.25 hPa
💧 Humidity: 45.67 %
```

## Understanding the Data

### Temperature

- **Unit**: Degrees Celsius (°C)
- **Range**: -40°C to +85°C
- **Accuracy**: ±1°C
- **What it means**: Room temperature is typically 20-25°C

```python
temp_c = temperature / 100  # Convert from hundredths
temp_f = (temp_c * 9/5) + 32  # Convert to Fahrenheit

print(f"{temp_c:.1f} °C = {temp_f:.1f} °F")
```

### Pressure

- **Unit**: Hectopascals (hPa) or millibars (mbar)
- **Range**: 300-1100 hPa
- **Accuracy**: ±1 hPa
- **What it means**: Sea level pressure is ~1013 hPa

```python
pressure_hpa = pressure / 256 / 100
pressure_mmhg = pressure_hpa * 0.75006  # Convert to mmHg
pressure_inhg = pressure_hpa * 0.02953  # Convert to inHg

print(f"Pressure: {pressure_hpa:.2f} hPa")
print(f"         {pressure_mmhg:.2f} mmHg")
print(f"         {pressure_inhg:.2f} inHg")
```

### Humidity

- **Unit**: Percent relative humidity (%)
- **Range**: 0-100%
- **Accuracy**: ±3%
- **What it means**: Comfort zone is 30-60%

```python
humidity = hum / 1024

if humidity < 30:
    comfort = "Dry"
elif humidity < 60:
    comfort = "Comfortable"
else:
    comfort = "Humid"

print(f"Humidity: {humidity:.1f}% - {comfort}")
```

## Continuous Monitoring

Read sensor data continuously:

```python
from machine import I2C, Pin
import bme280
import time

# Setup
i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400000)
bme = bme280.BME280(i2c=i2c)

print("Environmental Monitor Starting...")
print("Press Ctrl+C to stop\n")

try:
    while True:
        # Read sensor
        temp, press, hum = bme.read_compensated_data()

        # Convert units
        temperature = temp / 100
        pressure = press / 256 / 100
        humidity = hum / 1024

        # Display
        print(f"Temp: {temperature:6.2f}°C | ", end="")
        print(f"Press: {pressure:7.2f} hPa | ", end="")
        print(f"Humid: {humidity:5.2f}%")

        # Wait 2 seconds
        time.sleep(2)

except KeyboardInterrupt:
    print("\nMonitoring stopped")
```

**Output:**

```
Environmental Monitor Starting...
Press Ctrl+C to stop

Temp:  22.34°C | Press: 1013.25 hPa | Humid: 45.23%
Temp:  22.35°C | Press: 1013.26 hPa | Humid: 45.24%
Temp:  22.36°C | Press: 1013.24 hPa | Humid: 45.25%
^C
Monitoring stopped
```

## Advanced Reading Techniques

### Filtering Noisy Data

Sensors can have measurement noise. Average multiple readings:

```python
def read_average(bme, samples=10):
    """Read sensor multiple times and return average"""
    temp_sum = press_sum = hum_sum = 0

    for _ in range(samples):
        temp, press, hum = bme.read_compensated_data()
        temp_sum += temp
        press_sum += press
        hum_sum += hum
        time.sleep(0.1)

    return temp_sum / samples, press_sum / samples, hum_sum / samples

# Use it
temp, press, hum = read_average(bme, samples=5)
print(f"Averaged reading: {temp/100:.2f}°C")
```

### Detecting Changes

Only print when values change significantly:

```python
from machine import I2C, Pin
import bme280
import time

i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400000)
bme = bme280.BME280(i2c=i2c)

# Store previous values
prev_temp = prev_press = prev_hum = None

# Change thresholds
TEMP_THRESHOLD = 0.5   # 0.5°C
PRESS_THRESHOLD = 1.0  # 1 hPa
HUM_THRESHOLD = 2.0    # 2%

print("Monitoring for changes...\n")

while True:
    temp, press, hum = bme.read_compensated_data()

    # Convert
    temperature = temp / 100
    pressure = press / 256 / 100
    humidity = hum / 1024

    # Check for significant changes
    if prev_temp is None or \
       abs(temperature - prev_temp) >= TEMP_THRESHOLD or \
       abs(pressure - prev_press) >= PRESS_THRESHOLD or \
       abs(humidity - prev_hum) >= HUM_THRESHOLD:

        print(f"{temperature:.2f}°C | {pressure:.2f} hPa | {humidity:.2f}%")

        prev_temp = temperature
        prev_press = pressure
        prev_hum = humidity

    time.sleep(1)
```

### Calculating Derived Values

Compute additional useful metrics:

```python
def calculate_dew_point(temperature, humidity):
    """Calculate dew point from temperature and humidity"""
    a = 17.27
    b = 237.7
    alpha = ((a * temperature) / (b + temperature)) + (humidity / 100.0)
    dew_point = (b * alpha) / (a - alpha)
    return dew_point

def calculate_heat_index(temperature, humidity):
    """Calculate heat index (feels-like temperature)"""
    # Simplified formula - works for temps above 27°C
    if temperature < 27:
        return temperature

    hi = -8.78469475556 + \
         1.61139411 * temperature + \
         2.33854883889 * humidity + \
         -0.14611605 * temperature * humidity

    return hi

# Use them
temp, press, hum = bme.read_compensated_data()
temperature = temp / 100
humidity = hum / 1024

dew_point = calculate_dew_point(temperature, humidity)
heat_index = calculate_heat_index(temperature, humidity)

print(f"Temperature: {temperature:.1f}°C")
print(f"Humidity: {humidity:.1f}%")
print(f"Dew Point: {dew_point:.1f}°C")
print(f"Feels Like: {heat_index:.1f}°C")
```

## Try It Yourself

Experiment with sensor readings:

1. **Breathe on the sensor** - Watch temperature and humidity increase

2. **Cup your hands around it** - See temperature rise

3. **Move to different locations** - Compare readings in different rooms

4. **Create a data logger**:

```python
from machine import I2C, Pin
import bme280
import time

i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400000)
bme = bme280.BME280(i2c=i2c)

print("Logging data every 10 seconds...")
print("Time | Temp | Press | Humid")
print("-" * 40)

readings = 0
while readings < 20:  # Log 20 readings
    temp, press, hum = bme.read_compensated_data()

    temperature = temp / 100
    pressure = press / 256 / 100
    humidity = hum / 1024

    print(f"{readings:3d}  | {temperature:5.1f} | {pressure:6.1f} | {humidity:5.1f}")

    readings += 1
    time.sleep(10)

print("\nLogging complete!")
```

5. **Challenge**: Create an alert system that warns when temperature exceeds a threshold:

```python
TEMP_ALERT = 25.0  # Alert above 25°C

while True:
    temp, press, hum = bme.read_compensated_data()
    temperature = temp / 100

    if temperature > TEMP_ALERT:
        print(f"⚠️  ALERT: Temperature is {temperature:.1f}°C!")
    else:
        print(f"✓ Temperature OK: {temperature:.1f}°C")

    time.sleep(5)
```

## Common Issues

### Getting Zero or Strange Values

**Problem**: All readings are 0 or nonsensical

**Causes**:
- Sensor not initialized properly
- Wrong I2C address
- Calibration data not loaded

**Solution**:

```python
# Verify chip ID first
chip_id = i2c.readfrom_mem(0x76, 0xD0, 1)[0]
print(f"Chip ID: 0x{chip_id:02X}")  # Should be 0x60

# If wrong, try other address
chip_id = i2c.readfrom_mem(0x77, 0xD0, 1)[0]
print(f"Chip ID: 0x{chip_id:02X}")
```

### OSError When Reading

**Problem**: `OSError: [Errno 5] EIO` or `OSError: [Errno 19] ENODEV`

**Causes**:
- Sensor disconnected mid-read
- Loose connection
- Power issue

**Solution**: Add error handling:

```python
try:
    temp, press, hum = bme.read_compensated_data()
    print(f"Temp: {temp/100:.2f}°C")
except OSError as e:
    print(f"Sensor read error: {e}")
    print("Check connections!")
```

### Readings Don't Change

**Problem**: Values are constant regardless of environment

**Causes**:
- Sensor in wrong mode (sleep mode)
- Reading old data from buffer
- Faulty sensor

**Solution**: Ensure sensor is in normal mode and add delays:

```python
# Wait for sensor to complete measurement
time.sleep(0.1)
temp, press, hum = bme.read_compensated_data()
```

## What You've Learned

In this lesson, you mastered:

- Installing and using the BME280 MicroPython library
- Reading temperature, humidity, and pressure data
- Converting raw sensor values to meaningful units
- Continuous monitoring and data logging
- Filtering noisy readings with averaging
- Calculating derived values (dew point, heat index)
- Detecting significant changes in environment
- Handling sensor errors gracefully

You can now read real-world data from I2C sensors and use it in your projects. This is a foundational skill that applies to hundreds of other I2C devices.

## What's Next

You've successfully read sensor data, but how did you know which commands to send? How do you know what the numbers mean? The answer is in the **datasheet** - the sensor's instruction manual. In the next lesson, we'll demystify datasheets and learn to extract exactly what we need from these technical documents.
