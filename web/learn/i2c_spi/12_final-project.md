---
layout: lesson
title: Final Project - Environmental Monitoring Station
author: Kevin McAleer
type: page
cover: /learn/i2c_spi/assets/cover.jpg
date: 2026-01-15
previous: 11_common-issues.html
description: Build a complete environmental monitoring station combining I2C sensors,
  data logging, and everything learned in this course
percent: 100
duration: 11
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


![Environmental Monitor](/learn/i2c_spi/assets/final-project.jpg){:class="img-fluid w-100"}

## Your Capstone Project

Congratulations! You've mastered I2C and SPI communication. Now let's combine everything into a practical, working environmental monitoring station. This project demonstrates real-world application of the skills you've learned.

## Project Overview

**What we're building**:
A complete environmental monitoring system that:

- Reads temperature, humidity, and pressure from BME280 (I2C)
- Displays current readings on OLED display (I2C)
- Logs data to CSV file for analysis
- Monitors trends and alerts on abnormal conditions
- Runs continuously with error recovery

**Skills demonstrated**:
- I2C multi-device communication
- Sensor data acquisition
- Data processing and filtering
- File I/O and logging
- Display updates
- Error handling
- System architecture

## Hardware Requirements

### Essential Components

- **Raspberry Pi Pico** (or Pico W)
- **BME280 sensor** (I2C version)
- **128x64 OLED display** (I2C, SSD1306 chip)
- **Breadboard**
- **Jumper wires**
- **USB cable** for power and programming

### Optional Components

- **SD card module** (SPI) for larger storage
- **Real-time clock (DS3231)** for accurate timestamps
- **Buzzer or LED** for alerts
- **Button** for user interaction

## Circuit Diagram

### I2C Bus Wiring

All I2C devices share the same two wires:

```
Raspberry Pi Pico:
  GP8 (SDA) ──┬── BME280 SDA
              └── OLED SDA

  GP9 (SCL) ──┬── BME280 SCL
              └── OLED SCL

  3.3V ───────┬── BME280 VCC
              └── OLED VCC

  GND ────────┬── BME280 GND
              └── OLED GND
```

### Connection Summary

```python
# Pico to BME280
GP8  → SDA
GP9  → SCL
3.3V → VCC
GND  → GND

# Pico to OLED (same bus)
GP8  → SDA
GP9  → SCL
3.3V → VCC
GND  → GND
```

## Software Architecture

### Project Structure

```python
"""
Environmental Monitor System
├── main.py           # Main program loop
├── bme280.py         # BME280 driver (from earlier lessons)
├── ssd1306.py        # OLED display library
└── data_logger.py    # CSV logging utility
"""
```

### Core Components

1. **Sensor Manager** - Reads BME280 data
2. **Display Manager** - Updates OLED screen
3. **Data Logger** - Writes to CSV file
4. **Alert System** - Monitors thresholds
5. **Main Loop** - Coordinates everything

## Complete Implementation

### Step 1: Install Display Library

```python
# In Thonny: Tools → Manage packages
# Search for: micropython-ssd1306
# Or download from: https://github.com/stlehmann/micropython-ssd1306
```

### Step 2: BME280 Driver (Simplified)

```python
# bme280.py
from micropython import const
import time

BME280_ADDR = const(0x76)  # or 0x77

class BME280:
    """Simplified BME280 driver for environmental monitoring"""

    def __init__(self, i2c, addr=BME280_ADDR):
        self.i2c = i2c
        self.addr = addr

        # Verify device
        chip_id = self.i2c.readfrom_mem(self.addr, 0xD0, 1)[0]
        if chip_id != 0x60:
            raise RuntimeError("BME280 not found")

        # Configure sensor for normal operation
        self.i2c.writeto_mem(self.addr, 0xF2, b'\x01')  # Humidity oversampling x1
        self.i2c.writeto_mem(self.addr, 0xF4, b'\x27')  # Temp/press oversampling, normal mode
        self.i2c.writeto_mem(self.addr, 0xF5, b'\xA0')  # Standby 1000ms, filter off

        time.sleep_ms(10)  # Wait for config

    def read_data(self):
        """Read temperature, pressure, and humidity"""
        # Read all data registers (0xF7-0xFE)
        data = self.i2c.readfrom_mem(self.addr, 0xF7, 8)

        # Extract raw values
        press_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        hum_raw = (data[6] << 8) | data[7]

        # Simplified conversion (no calibration for brevity)
        # Real implementation should use calibration data
        temperature = temp_raw / 5120.0  # Approximate
        pressure = press_raw / 256.0     # Approximate
        humidity = hum_raw / 1024.0      # Approximate

        return temperature, pressure, humidity
```

> **Note**: For production use, implement proper calibration compensation as shown in earlier lessons.

### Step 3: Main Application

```python
# main.py
from machine import Pin, I2C
import time
import ssd1306
from bme280 import BME280

class EnvironmentalMonitor:
    """Complete environmental monitoring system"""

    def __init__(self):
        # Initialize I2C bus
        self.i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400000)
        print("I2C initialized")

        # Scan for devices
        devices = self.i2c.scan()
        print(f"Found devices: {[hex(d) for d in devices]}")

        # Initialize BME280 sensor
        if 0x76 in devices:
            self.sensor = BME280(self.i2c, addr=0x76)
            print("✓ BME280 initialized at 0x76")
        elif 0x77 in devices:
            self.sensor = BME280(self.i2c, addr=0x77)
            print("✓ BME280 initialized at 0x77")
        else:
            raise RuntimeError("BME280 not found")

        # Initialize OLED display (128x64)
        if 0x3C in devices:
            self.display = ssd1306.SSD1306_I2C(128, 64, self.i2c, addr=0x3C)
            print("✓ OLED display initialized at 0x3C")
        elif 0x3D in devices:
            self.display = ssd1306.SSD1306_I2C(128, 64, self.i2c, addr=0x3D)
            print("✓ OLED display initialized at 0x3D")
        else:
            self.display = None
            print("⚠️  OLED display not found - continuing without display")

        # Alert thresholds
        self.temp_low = 15.0    # °C
        self.temp_high = 30.0   # °C
        self.humidity_low = 30.0  # %
        self.humidity_high = 70.0 # %

        # Data logging
        self.log_file = "environment.csv"
        self.init_log_file()

        # Statistics
        self.reading_count = 0
        self.error_count = 0

    def init_log_file(self):
        """Create CSV file with headers"""
        try:
            with open(self.log_file, "w") as f:
                f.write("Timestamp,Temperature_C,Pressure_hPa,Humidity_%,Status\n")
            print(f"✓ Log file '{self.log_file}' created")
        except Exception as e:
            print(f"⚠️  Could not create log file: {e}")

    def read_sensors(self):
        """Read all sensor data with error handling"""
        try:
            temp, press, hum = self.sensor.read_data()
            self.reading_count += 1
            return temp, press, hum, True
        except Exception as e:
            self.error_count += 1
            print(f"Sensor error: {e}")
            return None, None, None, False

    def check_alerts(self, temp, hum):
        """Check if readings are within safe ranges"""
        alerts = []

        if temp < self.temp_low:
            alerts.append(f"LOW TEMP: {temp:.1f}°C")
        elif temp > self.temp_high:
            alerts.append(f"HIGH TEMP: {temp:.1f}°C")

        if hum < self.humidity_low:
            alerts.append(f"LOW HUMIDITY: {hum:.1f}%")
        elif hum > self.humidity_high:
            alerts.append(f"HIGH HUMIDITY: {hum:.1f}%")

        return alerts

    def update_display(self, temp, press, hum, alerts):
        """Update OLED with current readings"""
        if self.display is None:
            return

        self.display.fill(0)  # Clear screen

        # Title
        self.display.text("Env Monitor", 0, 0)

        # Readings
        self.display.text(f"Temp: {temp:.1f}C", 0, 16)
        self.display.text(f"Pres: {press:.0f}hPa", 0, 28)
        self.display.text(f"Hum:  {hum:.1f}%", 0, 40)

        # Alert indicator
        if alerts:
            self.display.text("! ALERT !", 0, 54)
        else:
            self.display.text("OK", 0, 54)

        self.display.show()

    def log_data(self, temp, press, hum, status):
        """Log data to CSV file"""
        try:
            timestamp = time.time()
            with open(self.log_file, "a") as f:
                f.write(f"{timestamp},{temp:.2f},{press:.2f},{hum:.2f},{status}\n")
        except Exception as e:
            print(f"Log error: {e}")

    def print_summary(self, temp, press, hum, alerts):
        """Print reading summary to console"""
        print("\n" + "="*50)
        print(f"Reading #{self.reading_count}")
        print(f"Temperature:  {temp:.2f}°C ({temp*9/5+32:.2f}°F)")
        print(f"Pressure:     {press:.2f} hPa")
        print(f"Humidity:     {hum:.2f}%")

        if alerts:
            print("\n⚠️  ALERTS:")
            for alert in alerts:
                print(f"  - {alert}")
        else:
            print("\n✓ All readings normal")

        print(f"\nErrors: {self.error_count}")
        print("="*50)

    def run(self, interval=5, duration=None):
        """
        Run monitoring loop

        Args:
            interval: Seconds between readings (default 5)
            duration: Total seconds to run (None = forever)
        """
        print("\n" + "="*50)
        print("Environmental Monitor Starting")
        print("="*50)
        print(f"Interval: {interval}s")
        print(f"Duration: {'Forever' if duration is None else f'{duration}s'}")
        print("Press Ctrl+C to stop")
        print()

        start_time = time.time()

        try:
            while True:
                # Read sensors
                temp, press, hum, success = self.read_sensors()

                if success:
                    # Check alerts
                    alerts = self.check_alerts(temp, hum)

                    # Update display
                    self.update_display(temp, press, hum, alerts)

                    # Log data
                    status = "ALERT" if alerts else "OK"
                    self.log_data(temp, press, hum, status)

                    # Print summary
                    self.print_summary(temp, press, hum, alerts)

                else:
                    print("⚠️  Sensor read failed - retrying next cycle")

                # Check duration
                if duration and (time.time() - start_time) >= duration:
                    break

                # Wait for next reading
                time.sleep(interval)

        except KeyboardInterrupt:
            print("\n\nMonitoring stopped by user")

        # Final statistics
        elapsed = time.time() - start_time
        print("\n" + "="*50)
        print("Monitoring Session Complete")
        print("="*50)
        print(f"Duration:      {elapsed:.1f} seconds")
        print(f"Readings:      {self.reading_count}")
        print(f"Errors:        {self.error_count}")
        print(f"Success rate:  {100*self.reading_count/(self.reading_count+self.error_count):.1f}%")
        print(f"Data logged to: {self.log_file}")
        print("="*50)

# Run the monitor
if __name__ == "__main__":
    monitor = EnvironmentalMonitor()
    monitor.run(interval=5)  # Read every 5 seconds
```

## Running the Project

### Initial Setup

1. **Wire hardware** following the circuit diagram
2. **Install libraries**:
   - `bme280.py` (from earlier lessons or create simplified version)
   - `ssd1306.py` (from MicroPython package manager)
3. **Upload `main.py`** to your Pico
4. **Run the program**

### Expected Output

```
I2C initialized
Found devices: ['0x3c', '0x76']
✓ BME280 initialized at 0x76
✓ OLED display initialized at 0x3C
✓ Log file 'environment.csv' created

==================================================
Environmental Monitor Starting
==================================================
Interval: 5s
Duration: Forever
Press Ctrl+C to stop

==================================================
Reading #1
Temperature:  22.45°C (72.41°F)
Pressure:     1013.25 hPa
Humidity:     45.67%

✓ All readings normal

Errors: 0
==================================================

[Continues every 5 seconds...]
```

### Display Output

OLED screen shows:

```
Env Monitor
Temp: 22.5C
Pres: 1013hPa
Hum:  45.7%
OK
```

## Enhancements and Challenges

### Easy Enhancements

1. **Add more sensors**:

```python
# Add a light sensor (BH1750)
if 0x23 in devices:
    self.light_sensor = BH1750(self.i2c)
    light = self.light_sensor.read_lux()
    print(f"Light: {light} lux")
```

2. **Implement data visualization**:

```python
# Add simple bar graph to display
def draw_bar(self, value, max_value, y_pos):
    bar_width = int((value / max_value) * 120)
    self.display.fill_rect(0, y_pos, bar_width, 8, 1)
```

3. **Add button control**:

```python
button = Pin(15, Pin.IN, Pin.PULL_UP)

if button.value() == 0:  # Button pressed
    self.display_mode = "stats"  # Switch display mode
```

### Intermediate Challenges

1. **Add WiFi data upload** (Pico W):

```python
import network
import urequests

def upload_to_cloud(temp, hum, press):
    """Send data to cloud service"""
    url = "https://api.example.com/data"
    data = {"temperature": temp, "humidity": hum, "pressure": press}
    urequests.post(url, json=data)
```

2. **Implement data smoothing**:

```python
class MovingAverage:
    def __init__(self, size=5):
        self.data = []
        self.size = size

    def add(self, value):
        self.data.append(value)
        if len(self.data) > self.size:
            self.data.pop(0)
        return sum(self.data) / len(self.data)
```

3. **Add SPI SD card logging**:

```python
from machine import SPI
import sdcard
import os

spi = SPI(1, baudrate=1000000, sck=Pin(10), mosi=Pin(11), miso=Pin(12))
sd = sdcard.SDCard(spi, Pin(13))
os.mount(sd, '/sd')

# Log to SD card instead of flash
log_file = '/sd/environment.csv'
```

### Advanced Challenges

1. **Create web dashboard** (Pico W):

```python
import socket

def create_server():
    """Serve readings via HTTP"""
    addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
    s = socket.socket()
    s.bind(addr)
    s.listen(1)

    while True:
        cl, addr = s.accept()
        temp, press, hum, _ = monitor.read_sensors()
        html = f"<h1>Temp: {temp:.1f}C</h1>"
        cl.send(html)
        cl.close()
```

2. **Implement MQTT publishing**:

```python
from umqtt.simple import MQTTClient

client = MQTTClient("pico", "mqtt.broker.com")
client.connect()
client.publish(b"sensors/temperature", str(temp))
```

3. **Add predictive alerts**:

```python
def predict_trend(readings):
    """Analyze if temperature is rising or falling"""
    if len(readings) < 5:
        return "STABLE"

    recent = readings[-5:]
    trend = (recent[-1] - recent[0]) / len(recent)

    if trend > 0.5:
        return "RISING"
    elif trend < -0.5:
        return "FALLING"
    else:
        return "STABLE"
```

## Troubleshooting

### Display Not Updating

```python
# Add debug output
def update_display(self, temp, press, hum, alerts):
    if self.display is None:
        print("Display is None - check initialization")
        return

    try:
        self.display.fill(0)
        # ... display code ...
        self.display.show()
        print("Display updated successfully")
    except Exception as e:
        print(f"Display error: {e}")
```

### Sensor Readings Unstable

```python
# Add averaging
class SensorWithAverage:
    def __init__(self, sensor):
        self.sensor = sensor
        self.readings = []

    def read_averaged(self, samples=3):
        temps, presses, hums = [], [], []

        for _ in range(samples):
            t, p, h = self.sensor.read_data()
            temps.append(t)
            presses.append(p)
            hums.append(h)
            time.sleep(0.1)

        return sum(temps)/samples, sum(presses)/samples, sum(hums)/samples
```

## What You've Learned

In this final project, you demonstrated:

- **System integration** - Combining multiple I2C devices on one bus
- **Real-world application** - Building a practical monitoring station
- **Data management** - Logging to files and managing storage
- **User interface** - Displaying information on OLED screen
- **Error handling** - Robust operation with retries and statistics
- **Software architecture** - Clean, modular code structure
- **Hardware interfacing** - Working with real sensors and displays
- **Problem solving** - Debugging and enhancing a complex system

## Course Completion

Congratulations! You've completed "Talking to the World - Working with I2C and SPI"!

### What You Accomplished

- ✓ Mastered I2C protocol and implementation
- ✓ Mastered SPI protocol and implementation
- ✓ Read real sensors (BME280, MAX31855)
- ✓ Controlled displays via I2C
- ✓ Understood datasheets and register maps
- ✓ Built robust error handling
- ✓ Created a complete working project

### Next Steps

**Expand this project**:
- Add more sensors (light, motion, air quality)
- Implement cloud connectivity
- Create mobile app interface
- Build weatherproof enclosure

**Explore related topics**:
- [Adding WiFi Communication](/learn/micropython-wifi/) - Network your sensors
- [Display Projects](/learn/displays/) - Advanced display techniques
- [IoT Weather Station](/learn/weather-station/) - Complete weather station build

**Share your project**:
- Post on social media
- Write a blog post
- Help others in the community

## Course Resources

**Code repository**: All examples from this course
**Datasheets**: BME280, MAX31855, SSD1306
**Community forum**: Get help and share projects
**Video tutorials**: Watch the companion video series

Thank you for taking this journey into I2C and SPI communication. You now have the skills to connect your Raspberry Pi Pico to hundreds of sensors and devices. Keep building, keep learning, and most importantly - have fun!

---

> **Course Progress**: Lesson 12 of 12 - COMPLETE!
>
> **Previous**: [Common Issues and Troubleshooting](/learn/i2c_spi/11_common-issues.html)
>
> **Continue Learning**: [Explore More Courses](/learn/)
