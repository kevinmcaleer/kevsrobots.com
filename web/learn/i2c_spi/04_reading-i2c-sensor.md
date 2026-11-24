---
layout: lesson
title: Reading I2C Sensors
author: Kevin McAleer
type: page
cover: /learn/i2c_spi/assets/banners/04_reading-i2c-sensor.jpg
date: 2026-01-15
previous: 03_scanning-i2c-devices.html
next: 05_demystifying-datasheets.html
description: Learn how to read temperature, humidity, and pressure data from the BME280
  sensor using I2C
percent: 42
duration: 14
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

By the end of this lesson you will be able to save a BME280 driver on your Pico and read temperature, pressure and humidity from it over I2C.

## Reading Real Sensor Data

Now comes the exciting part - reading actual environmental data from your BME280 sensor! This lesson teaches you how to communicate with I2C sensors, use existing libraries, and understand what's happening behind the scenes.

## Two Approaches to Reading Sensors

You have two options when working with I2C sensors:

### 1. Using a Driver (Recommended for Projects)

A driver file handles the complex communication details:

```python
from machine import I2C, Pin
import bme280

# Initialize I2C
i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400000)

# Create BME280 object (use addr=0x77 if your scan showed 0x77)
bme = bme280.BME280(i2c)

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

> **Recommendation**: Use a driver for real projects, but understanding manual communication helps you debug and work with any sensor. (The first block above only works once `bme280.py` is saved on the Pico, which we do next.)

## Getting the BME280 Driver onto the Pico

MicroPython does not include a BME280 driver in its firmware, so `import bme280` fails with `ImportError: no module named 'bme280'` until you save a file called `bme280.py` on the Pico. Here is a complete driver you can use. It checks the chip ID, reads the factory calibration numbers stored inside the chip, and applies the integer formulas from the Bosch datasheet, so the values it gives you are real temperatures, pressures and humidities.

```python
# bme280.py - a small, complete BME280 driver for MicroPython (I2C)
import struct
import time

BME280_ADDR = 0x76          # 0x77 if the SDO pin is tied to 3.3V

REG_ID = 0xD0
REG_RESET = 0xE0
REG_CTRL_HUM = 0xF2
REG_STATUS = 0xF3
REG_CTRL_MEAS = 0xF4
REG_CONFIG = 0xF5
REG_DATA = 0xF7


class BME280:
    def __init__(self, i2c, addr=BME280_ADDR):
        self.i2c = i2c
        self.addr = addr

        chip_id = self.i2c.readfrom_mem(addr, REG_ID, 1)[0]
        if chip_id != 0x60:
            raise RuntimeError("Not a BME280, chip ID is 0x%02X" % chip_id)

        self._load_calibration()
        self.t_fine = 0

    def _load_calibration(self):
        """Read the factory calibration numbers stored inside the chip."""
        # 0x88..0x9F: temperature and pressure coefficients (24 bytes)
        (self.T1, self.T2, self.T3,
         self.P1, self.P2, self.P3, self.P4, self.P5,
         self.P6, self.P7, self.P8, self.P9) = struct.unpack(
            "<HhhHhhhhhhhh", self.i2c.readfrom_mem(self.addr, 0x88, 24))
        # 0xA1: humidity coefficient H1
        self.H1 = self.i2c.readfrom_mem(self.addr, 0xA1, 1)[0]
        # 0xE1..0xE7: the other humidity coefficients (7 bytes)
        e = self.i2c.readfrom_mem(self.addr, 0xE1, 7)
        self.H2 = struct.unpack("<h", e[0:2])[0]
        self.H3 = e[2]
        h4 = (e[3] << 4) | (e[4] & 0x0F)
        h5 = (e[5] << 4) | (e[4] >> 4)
        self.H4 = h4 - 4096 if h4 > 2047 else h4
        self.H5 = h5 - 4096 if h5 > 2047 else h5
        self.H6 = e[6] - 256 if e[6] > 127 else e[6]

    def read_raw_data(self):
        """Take one reading and return the raw (temp, pressure, humidity)."""
        # Forced mode: measure once, then go back to sleep.
        self.i2c.writeto_mem(self.addr, REG_CTRL_HUM, b"\x01")   # humidity x1
        self.i2c.writeto_mem(self.addr, REG_CTRL_MEAS, b"\x25")  # T x1, P x1, forced
        time.sleep_ms(12)                                        # conversion takes under 10 ms
        d = self.i2c.readfrom_mem(self.addr, REG_DATA, 8)
        press_raw = (d[0] << 12) | (d[1] << 4) | (d[2] >> 4)
        temp_raw = (d[3] << 12) | (d[4] << 4) | (d[5] >> 4)
        hum_raw = (d[6] << 8) | d[7]
        return temp_raw, press_raw, hum_raw

    def read_compensated_data(self):
        """Return (temperature, pressure, humidity) in fixed-point units.

        temperature: hundredths of a degree C   (2245 means 22.45 C)
        pressure:    Pa x 256                   (divide by 256 for Pa)
        humidity:    %RH x 1024                 (divide by 1024 for %RH)
        These are the integer formulas from the Bosch datasheet.
        """
        adc_t, adc_p, adc_h = self.read_raw_data()

        # Temperature
        var1 = (((adc_t >> 3) - (self.T1 << 1)) * self.T2) >> 11
        var2 = (((((adc_t >> 4) - self.T1) * ((adc_t >> 4) - self.T1)) >> 12) * self.T3) >> 14
        self.t_fine = var1 + var2
        temp = (self.t_fine * 5 + 128) >> 8

        # Pressure
        var1 = self.t_fine - 128000
        var2 = var1 * var1 * self.P6
        var2 = var2 + ((var1 * self.P5) << 17)
        var2 = var2 + (self.P4 << 35)
        var1 = ((var1 * var1 * self.P3) >> 8) + ((var1 * self.P2) << 12)
        var1 = (((1 << 47) + var1) * self.P1) >> 33
        if var1 == 0:
            press = 0
        else:
            p = 1048576 - adc_p
            p = (((p << 31) - var2) * 3125) // var1
            var1 = (self.P9 * (p >> 13) * (p >> 13)) >> 25
            var2 = (self.P8 * p) >> 19
            press = ((p + var1 + var2) >> 8) + (self.P7 << 4)

        # Humidity
        h = self.t_fine - 76800
        h = (((((adc_h << 14) - (self.H4 << 20) - (self.H5 * h)) + 16384) >> 15) *
             (((((((h * self.H6) >> 10) * (((h * self.H3) >> 11) + 32768)) >> 10)
                + 2097152) * self.H2 + 8192) >> 14))
        h = h - (((((h >> 15) * (h >> 15)) >> 7) * self.H1) >> 4)
        h = max(0, min(h, 419430400))
        hum = h >> 12

        return temp, press, hum

    def read(self):
        """Return (temperature in C, pressure in hPa, humidity in %RH)."""
        t, p, h = self.read_compensated_data()
        return t / 100, p / 256 / 100, h / 1024
```
{: .no-snakie }

**How to copy it to the Pico:**

1. Open a new file in Thonny and paste in the code above
2. Choose **File, Save as**, and pick **Raspberry Pi Pico** when asked where to save
3. Name the file `bme280.py` and save it in the top folder (the root) of the Pico

The raw numbers need calibration because every BME280 is slightly different from the factory: the chip stores about 30 correction values, and the datasheet formulas combine them with the raw readings. That is the part earlier "simplified" examples skip, and it is why raw values on their own are meaningless.

> **Note**: The driver uses forced mode, which tells the sensor to take one reading and go back to sleep. Lesson 6 explains the modes. If you would rather use a community driver, Robert Hammelrath's `robert-hh/BME280` on GitHub returns values in the same units, but check its README for the constructor arguments, because they differ slightly from the class above.

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

    print(f"Temperature: {temperature_c:.2f} °C")
    print(f"Pressure: {pressure_hpa:.2f} hPa")
    print(f"Humidity: {humidity_pct:.2f} %")
```

**Example output:**

```
Scanning for BME280...
BME280 found!
Using address 0x76

Reading sensor data...

Temperature: 22.45 °C
Pressure: 1013.25 hPa
Humidity: 45.67 %
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
import math

def calculate_dew_point(temperature, humidity):
    """Calculate dew point from temperature (C) and relative humidity (%)"""
    a = 17.27
    b = 237.7
    alpha = ((a * temperature) / (b + temperature)) + math.log(humidity / 100.0)
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

# Use them (this snippet carries on from the earlier code, where bme was created)
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
        print(f"[WARNING] ALERT: Temperature is {temperature:.1f}°C!")
    else:
        print(f"[OK] Temperature: {temperature:.1f}°C")

    time.sleep(5)
```

## Common Issues

- **Problem**: `ImportError: no module named 'bme280'`
- **Solution**: Save the driver on the Pico as `bme280.py` (File, Save as, Raspberry Pi Pico), in the top folder
- **Why**: The BME280 driver is not part of the MicroPython firmware, so the Pico only finds it if the file is on its own storage

- **Problem**: `RuntimeError: Not a BME280, chip ID is 0x58` (or 0x00)
- **Solution**: 0x58 means you have a BMP280, which has no humidity sensor, so use a BMP280 driver. 0x00 means nothing answered, so check the wiring and the address (0x76 or 0x77)
- **Why**: The driver reads the chip ID register (0xD0) first, and the BME280 always answers 0x60

- **Problem**: `OSError: [Errno 5] EIO` while reading
- **Solution**: Check for a loose wire, then add `try/except OSError` around the read and retry:

```python
try:
    temp, press, hum = bme.read_compensated_data()
    print(f"Temp: {temp/100:.2f}°C")
except OSError as e:
    print(f"Sensor read error: {e}")
    print("Check connections!")
```
{: .no-snakie }

- **Why**: The sensor stopped acknowledging, which is almost always a wiring or power problem

- **Problem**: The temperature reads a degree or two high
- **Solution**: Keep the sensor away from the Pico, regulators and your hand, and let it settle for a minute
- **Why**: The BME280 measures the air next to it, and nearby electronics warm that air

## Check your understanding

{% quiz %}
? What does the BME280 chip ID register (0xD0) return?
- [ ] 0x76
- [x] 0x60
- [ ] 0x58
> 0x76 is a common I2C address, and 0x58 is the BMP280's ID. The BME280 always reports 0x60.

? Why does the driver read calibration data from the chip?
- [ ] To set the I2C address
- [x] Because the raw readings only become real units after the factory correction values are applied
- [ ] To speed up the bus
> Each chip is calibrated at the factory and stores its own correction numbers.

? The driver returns pressure as `press`. How do you turn it into hectopascals?
- [ ] `press / 100`
- [x] `press / 256 / 100`
- [ ] `press * 256`
> Pressure comes back in Pa multiplied by 256, so divide by 256 for pascals and by 100 for hPa.
{% endquiz %}

## What You've Learned

In this lesson, you mastered:

- Saving a complete BME280 driver on the Pico and using it
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
