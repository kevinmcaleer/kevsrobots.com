---
layout: lesson
title: SPI Sensor Example
author: Kevin McAleer
type: page
cover: /learn/i2c_spi/assets/cover.jpg
date: 2026-01-15
previous: 08_spi-hardware-setup.html
next: 10_i2c-vs-spi.html
description: Build a complete temperature monitoring system using the MAX31855 thermocouple
  amplifier
percent: 77
duration: 9
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


<!-- ![Temperature Monitoring](/learn/i2c_spi/assets/max31855-monitor.jpg){:class="img-fluid w-100"} -->

## Building a Temperature Monitor

Let's build a complete, production-ready temperature monitoring system using the MAX31855 thermocouple amplifier. This lesson covers reading data, error handling, calibration, and creating a robust sensor class.

## Understanding the MAX31855

The MAX31855 reads K-type thermocouples and provides:

- **Temperature range**: -200°C to +1350°C
- **Resolution**: 0.25°C
- **Accuracy**: ±2°C (up to 700°C)
- **Cold junction compensation**: Built-in reference temperature
- **SPI interface**: Simple, fast communication

Perfect for monitoring ovens, 3D printer hotends, engines, and high-temperature processes.

## MAX31855 Data Format

The MAX31855 sends 32 bits (4 bytes) of data:

```
Bit 31-18: Thermocouple temperature (14-bit signed, 0.25°C per LSB)
Bit 17:    Reserved
Bit 16:    Fault indicator (1 = error)
Bit 15-4:  Internal temperature (12-bit signed, 0.0625°C per LSB)
Bit 3:     Reserved
Bit 2:     SCV (Short to VCC) fault
Bit 1:     SCG (Short to GND) fault
Bit 0:     OC (Open Circuit) fault
```

## Basic MAX31855 Class

Let's create a robust sensor class:

```python
from machine import Pin, SPI
import time

class MAX31855:
    """Driver for MAX31855 thermocouple amplifier"""

    def __init__(self, spi, cs_pin):
        """
        Initialize MAX31855

        Args:
            spi: SPI object (already configured)
            cs_pin: Pin number for chip select
        """
        self.spi = spi
        self.cs = Pin(cs_pin, Pin.OUT)
        self.cs.value(1)  # Start deselected

        # Verify device responds
        self._verify_device()

    def _verify_device(self):
        """Check if MAX31855 is connected"""
        try:
            temp = self.read_temperature()
            if temp is None:
                print("⚠️  Warning: MAX31855 might not be connected properly")
            else:
                print(f"✓ MAX31855 initialized ({temp:.2f}°C)")
        except Exception as e:
            print(f"❌ MAX31855 initialization failed: {e}")

    def read_raw(self):
        """Read 32-bit raw data from MAX31855"""
        self.cs.value(0)
        time.sleep_us(1)  # Chip select setup time

        # Read 4 bytes
        data = bytearray(4)
        self.spi.readinto(data)

        self.cs.value(1)
        time.sleep_us(1)  # Chip select hold time

        # Combine into 32-bit value
        raw = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]
        return raw

    def check_errors(self, raw_data):
        """
        Check for thermocouple errors

        Returns:
            dict: Error status {has_error: bool, error_type: str}
        """
        error_bit = (raw_data >> 16) & 0x01

        if not error_bit:
            return {"has_error": False, "error_type": None}

        # Check specific faults
        scv_fault = (raw_data >> 2) & 0x01  # Short to VCC
        scg_fault = (raw_data >> 1) & 0x01  # Short to GND
        oc_fault = raw_data & 0x01          # Open circuit

        if oc_fault:
            error_type = "Open circuit - thermocouple not connected"
        elif scg_fault:
            error_type = "Short to ground"
        elif scv_fault:
            error_type = "Short to VCC"
        else:
            error_type = "Unknown fault"

        return {"has_error": True, "error_type": error_type}

    def read_temperature(self):
        """
        Read thermocouple temperature

        Returns:
            float: Temperature in Celsius, or None if error
        """
        raw = self.read_raw()

        # Check for errors
        error_status = self.check_errors(raw)
        if error_status["has_error"]:
            print(f"❌ Thermocouple error: {error_status['error_type']}")
            return None

        # Extract temperature (bits 31:18)
        temp_raw = (raw >> 18) & 0x3FFF

        # Check sign bit (bit 13)
        if temp_raw & 0x2000:
            # Negative temperature - two's complement
            temp_raw = -((~temp_raw & 0x3FFF) + 1)

        # Convert to Celsius (0.25°C per LSB)
        temperature = temp_raw * 0.25

        return temperature

    def read_internal_temperature(self):
        """
        Read MAX31855 internal (cold junction) temperature

        Returns:
            float: Internal temperature in Celsius
        """
        raw = self.read_raw()

        # Extract internal temp (bits 15:4)
        internal_raw = (raw >> 4) & 0xFFF

        # Check sign bit (bit 11)
        if internal_raw & 0x800:
            internal_raw = -((~internal_raw & 0xFFF) + 1)

        # Convert to Celsius (0.0625°C per LSB)
        internal_temp = internal_raw * 0.0625

        return internal_temp

# Initialize
spi = SPI(0, baudrate=5000000, polarity=0, phase=0,
          sck=Pin(2), mosi=Pin(3), miso=Pin(4))

sensor = MAX31855(spi, cs_pin=5)

# Read temperature
temp = sensor.read_temperature()
if temp is not None:
    print(f"Temperature: {temp:.2f}°C")

internal = sensor.read_internal_temperature()
print(f"Internal temp: {internal:.2f}°C")
```

## Continuous Monitoring

Create a real-time temperature monitor:

```python
from machine import Pin, SPI
import time

# ... (MAX31855 class from above)

def monitor_temperature(sensor, duration_seconds=60, interval_seconds=2):
    """
    Monitor temperature continuously

    Args:
        sensor: MAX31855 instance
        duration_seconds: How long to monitor
        interval_seconds: Time between readings
    """
    print("\n" + "="*50)
    print("Temperature Monitor")
    print("="*50)
    print(f"Reading every {interval_seconds}s for {duration_seconds}s")
    print("Press Ctrl+C to stop early\n")

    start_time = time.time()
    reading_count = 0

    try:
        while time.time() - start_time < duration_seconds:
            temp = sensor.read_temperature()

            if temp is not None:
                reading_count += 1
                temp_f = temp * 9/5 + 32  # Convert to Fahrenheit

                print(f"[{reading_count:3d}] {temp:7.2f}°C | {temp_f:7.2f}°F")
            else:
                print(f"[{reading_count:3d}] ERROR - Check thermocouple connection")

            time.sleep(interval_seconds)

    except KeyboardInterrupt:
        print("\n\nMonitoring stopped by user")

    elapsed = time.time() - start_time
    print(f"\nCompleted {reading_count} readings in {elapsed:.1f} seconds")

# Run monitor
spi = SPI(0, baudrate=5000000, polarity=0, phase=0,
          sck=Pin(2), mosi=Pin(3), miso=Pin(4))
sensor = MAX31855(spi, cs_pin=5)

monitor_temperature(sensor, duration_seconds=30, interval_seconds=1)
```

**Example output**:

```
==================================================
Temperature Monitor
==================================================
Reading every 1s for 30s
Press Ctrl+C to stop early

[  1]   22.50°C |   72.50°F
[  2]   22.75°C |   72.95°F
[  3]   23.00°C |   73.40°F
[  4]   23.25°C |   73.85°F
...
^C

Monitoring stopped by user

Completed 15 readings in 15.2 seconds
```

## Temperature Alerts

Add threshold alerts for safety monitoring:

```python
class MAX31855_WithAlerts(MAX31855):
    """MAX31855 with temperature alerts"""

    def __init__(self, spi, cs_pin, low_threshold=0, high_threshold=100):
        super().__init__(spi, cs_pin)
        self.low_threshold = low_threshold
        self.high_threshold = high_threshold
        self.alert_active = False

    def read_with_alert(self):
        """Read temperature and check thresholds"""
        temp = self.read_temperature()

        if temp is None:
            return None, "ERROR"

        # Check thresholds
        if temp < self.low_threshold:
            status = "LOW"
            if not self.alert_active:
                print(f"\n⚠️  LOW TEMPERATURE ALERT: {temp:.2f}°C\n")
                self.alert_active = True
        elif temp > self.high_threshold:
            status = "HIGH"
            if not self.alert_active:
                print(f"\n🔥 HIGH TEMPERATURE ALERT: {temp:.2f}°C\n")
                self.alert_active = True
        else:
            status = "OK"
            self.alert_active = False

        return temp, status

# Example: Monitor oven temperature
spi = SPI(0, baudrate=5000000, polarity=0, phase=0,
          sck=Pin(2), mosi=Pin(3), miso=Pin(4))

# Alert if below 150°C or above 250°C
oven_sensor = MAX31855_WithAlerts(spi, cs_pin=5,
                                   low_threshold=150,
                                   high_threshold=250)

print("Monitoring oven temperature (150-250°C safe range)...\n")

for i in range(20):
    temp, status = oven_sensor.read_with_alert()

    if temp is not None:
        indicator = "✓" if status == "OK" else "⚠️"
        print(f"{indicator} [{status:5s}] {temp:6.2f}°C")
    else:
        print("❌ [ERROR] Thermocouple fault")

    time.sleep(2)
```

## Data Logging

Log temperature data to a file:

```python
from machine import Pin, SPI, RTC
import time

# ... (MAX31855 class from above)

def log_temperature(sensor, filename="temp_log.csv", duration_seconds=3600):
    """
    Log temperature data to CSV file

    Args:
        sensor: MAX31855 instance
        filename: Output file name
        duration_seconds: How long to log (default 1 hour)
    """
    print(f"Logging temperature to {filename}...")
    print(f"Duration: {duration_seconds} seconds")

    # Create/open log file
    with open(filename, "w") as f:
        # Write header
        f.write("Timestamp,Temperature_C,Temperature_F,Internal_C\n")

        start_time = time.time()
        count = 0

        try:
            while time.time() - start_time < duration_seconds:
                # Read temperatures
                temp_c = sensor.read_temperature()
                internal_c = sensor.read_internal_temperature()

                if temp_c is not None:
                    temp_f = temp_c * 9/5 + 32
                    timestamp = time.time()

                    # Write to file
                    f.write(f"{timestamp},{temp_c:.2f},{temp_f:.2f},{internal_c:.2f}\n")
                    f.flush()  # Ensure data is written

                    count += 1
                    if count % 10 == 0:
                        print(f"Logged {count} readings ({temp_c:.2f}°C)")

                time.sleep(1)  # Log every second

        except KeyboardInterrupt:
            print("\nLogging stopped by user")

    print(f"\nLogged {count} readings to {filename}")

# Usage
spi = SPI(0, baudrate=5000000, polarity=0, phase=0,
          sck=Pin(2), mosi=Pin(3), miso=Pin(4))
sensor = MAX31855(spi, cs_pin=5)

log_temperature(sensor, filename="oven_temp.csv", duration_seconds=60)
```

## Advanced: Moving Average Filter

Smooth noisy readings:

```python
class MAX31855_Filtered(MAX31855):
    """MAX31855 with moving average filter"""

    def __init__(self, spi, cs_pin, filter_size=5):
        super().__init__(spi, cs_pin)
        self.filter_size = filter_size
        self.readings = []

    def read_filtered_temperature(self):
        """Read temperature with moving average filter"""
        temp = self.read_temperature()

        if temp is None:
            return None

        # Add to readings list
        self.readings.append(temp)

        # Keep only last N readings
        if len(self.readings) > self.filter_size:
            self.readings.pop(0)

        # Calculate average
        avg_temp = sum(self.readings) / len(self.readings)

        return avg_temp

# Compare raw vs filtered
spi = SPI(0, baudrate=5000000, polarity=0, phase=0,
          sck=Pin(2), mosi=Pin(3), miso=Pin(4))

sensor = MAX31855_Filtered(spi, cs_pin=5, filter_size=5)

print("Comparing raw vs filtered readings:\n")
print("Raw     | Filtered")
print("--------+---------")

for i in range(20):
    raw = sensor.read_temperature()
    filtered = sensor.read_filtered_temperature()

    if raw is not None:
        print(f"{raw:6.2f}°C | {filtered:6.2f}°C")

    time.sleep(0.5)
```

## Try It Yourself

Experiment with the MAX31855:

1. **Touch the thermocouple** - Watch temperature increase

2. **Monitor your environment**:

```python
sensor = MAX31855(spi, cs_pin=5)

print("24-hour temperature monitoring")
log_temperature(sensor, duration_seconds=86400)  # 24 hours
```

3. **Create a temperature controller**:

```python
def simple_controller(sensor, target_temp, heater_pin):
    """Simple on/off temperature controller"""
    heater = Pin(heater_pin, Pin.OUT)

    while True:
        temp = sensor.read_temperature()

        if temp is None:
            heater.value(0)  # Safety: turn off if error
            print("ERROR - Heater OFF")
        elif temp < target_temp:
            heater.value(1)  # Turn on heater
            print(f"{temp:.1f}°C - Heating ON")
        else:
            heater.value(0)  # Turn off heater
            print(f"{temp:.1f}°C - Heating OFF")

        time.sleep(1)

# Use with caution - add proper safety features!
# simple_controller(sensor, target_temp=50, heater_pin=10)
```

4. **Challenge**: Calculate rate of temperature change:

```python
def monitor_temp_rate(sensor):
    """Monitor temperature change rate (°C per minute)"""
    prev_temp = sensor.read_temperature()
    prev_time = time.time()

    time.sleep(10)  # Wait 10 seconds

    while True:
        curr_temp = sensor.read_temperature()
        curr_time = time.time()

        if curr_temp is not None and prev_temp is not None:
            # Calculate rate in °C per minute
            delta_temp = curr_temp - prev_temp
            delta_time = (curr_time - prev_time) / 60  # Convert to minutes
            rate = delta_temp / delta_time

            print(f"Temp: {curr_temp:6.2f}°C | Rate: {rate:+.2f}°C/min")

        prev_temp = curr_temp
        prev_time = curr_time

        time.sleep(10)

monitor_temp_rate(sensor)
```

## Error Handling Best Practices

Always handle errors gracefully:

```python
def safe_temperature_read(sensor, max_retries=3):
    """Read temperature with retry logic"""
    for attempt in range(max_retries):
        try:
            temp = sensor.read_temperature()

            if temp is not None:
                return temp
            else:
                print(f"Attempt {attempt + 1}: Thermocouple fault")

        except Exception as e:
            print(f"Attempt {attempt + 1}: {e}")

        time.sleep(0.5)

    print("Failed to read temperature after maximum retries")
    return None

# Use it
temp = safe_temperature_read(sensor)
if temp is not None:
    print(f"Temperature: {temp:.2f}°C")
else:
    print("Unable to read sensor - check connections")
```

## What You've Learned

In this lesson, you mastered:

- Creating a complete MAX31855 driver class
- Reading thermocouple temperature via SPI
- Detecting and handling thermocouple faults
- Building continuous monitoring systems
- Implementing temperature alerts and thresholds
- Logging data to CSV files
- Filtering noisy readings with moving averages
- Error handling and retry logic
- Building practical temperature control systems

You now have production-ready code for working with SPI sensors. These patterns apply to many other SPI devices.

## What's Next

You've mastered both I2C (BME280) and SPI (MAX31855). But how do you choose between them? In the next lesson, we'll compare the protocols side-by-side and learn when to use each for your projects.
