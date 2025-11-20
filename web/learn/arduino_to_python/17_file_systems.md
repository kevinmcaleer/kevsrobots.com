---
layout: lesson
title: File Systems - A New Capability in Python
author: Kevin McAleer
type: page
cover: /learn/arduino_to_python/assets/file_systems.jpg
date: 2025-01-20
previous: 16_libraries_vs_modules.html
next: 18_understanding_dual_processor.html
description: Reading and writing files for data logging and configuration
percent: 54
duration: 10
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


![File Systems cover image](assets/file_systems.jpg){:class="cover"}

## Introduction

Arduino boards typically don't have a file system. Sure, you can add an SD card module, but that's extra hardware and complexity. Your Arduino sketch exists in flash memory, and that's pretty much it.

MicroPython boards have a built-in file system on their flash storage. You can create, read, write, and delete files just like on a computer. This opens up powerful capabilities: data logging, configuration files, storing calibration data, and more.

In this lesson, you'll learn how to work with files in MicroPython, understand the file system structure, and build practical applications like data loggers and configuration managers.

## The Arduino Way: No File System

### Arduino (Flash + EEPROM Only)
```cpp
// Your code is in flash memory
// Variables are in SRAM (lost on power off)

// Option 1: EEPROM for persistent storage (very limited!)
#include <EEPROM.h>

void setup() {
  // Write single value to EEPROM
  int value = 42;
  EEPROM.write(0, value);

  // Read it back
  int stored = EEPROM.read(0);
}

// Option 2: SD card module (requires extra hardware)
#include <SD.h>

void setup() {
  if (!SD.begin(4)) {
    Serial.println("SD card failed!");
    return;
  }

  File dataFile = SD.open("log.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println("Data");
    dataFile.close();
  }
}
```

**Arduino limitations:**
- EEPROM: Only 1-4 KB, limited write cycles (~100,000)
- SD card: Requires external module, wiring, power
- No built-in file system

### MicroPython (Built-In File System!)
```python
# Write to file
with open('log.txt', 'w') as f:
    f.write('Data\n')

# Read from file
with open('log.txt', 'r') as f:
    data = f.read()
    print(data)

# That's it! No extra hardware needed!
```

**MicroPython advantages:**
- Built-in flash file system (typically 1-2 MB)
- Standard Python file operations
- No extra hardware required
- Can store thousands of files

## File System Structure

### Exploring the File System

```python
import os

# List files in root directory
print(os.listdir('/'))
# Output: ['boot.py', 'main.py', 'lib', 'data']

# Check if file exists
if 'config.json' in os.listdir():
    print("Config file found")

# Get file info
stat = os.stat('main.py')
print(f"Size: {stat[6]} bytes")

# Create directory
os.mkdir('data')

# Remove file
os.remove('old_log.txt')

# Remove directory (must be empty)
os.rmdir('old_folder')

# Rename file
os.rename('old.txt', 'new.txt')

# Get current working directory
print(os.getcwd())  # '/'
```

## Reading Files

### Basic File Reading

```python
# Read entire file as string
with open('data.txt', 'r') as f:
    content = f.read()
    print(content)

# Read file line by line
with open('data.txt', 'r') as f:
    for line in f:
        print(line.strip())  # strip() removes newline

# Read all lines into list
with open('data.txt', 'r') as f:
    lines = f.readlines()
    print(f"File has {len(lines)} lines")

# Read specific number of bytes
with open('data.txt', 'r') as f:
    chunk = f.read(100)  # Read first 100 bytes
```

### Error Handling

```python
try:
    with open('config.json', 'r') as f:
        data = f.read()
except OSError:
    print("File not found, using defaults")
    data = "{}"
```

## Writing Files

### Basic File Writing

```python
# Write string to file (overwrites existing)
with open('log.txt', 'w') as f:
    f.write('Temperature: 25.3°C\n')

# Append to file (adds to end)
with open('log.txt', 'a') as f:
    f.write('Humidity: 65%\n')

# Write multiple lines
lines = ['Line 1\n', 'Line 2\n', 'Line 3\n']
with open('output.txt', 'w') as f:
    f.writelines(lines)
```

### Why use `with` Statement?

```python
# Without 'with' - must manually close
f = open('data.txt', 'w')
f.write('data')
f.close()  # Easy to forget!

# With 'with' - automatically closes
with open('data.txt', 'w') as f:
    f.write('data')
# File automatically closed here, even if error occurs
```

## Real-World Example: Temperature Logger

### Arduino (SD Card Required)
```cpp
#include <SD.h>

const int chipSelect = 4;

void setup() {
  Serial.begin(9600);

  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    return;
  }
}

void loop() {
  float temp = analogRead(A0) * 0.48828125;  // Convert to Celsius

  File dataFile = SD.open("temp_log.csv", FILE_WRITE);

  if (dataFile) {
    dataFile.print(millis());
    dataFile.print(",");
    dataFile.println(temp, 1);
    dataFile.close();

    Serial.println("Logged");
  } else {
    Serial.println("Error opening file");
  }

  delay(60000);  // Log every minute
}
```

### MicroPython (Built-In)
```python
from machine import ADC, Pin
import time

# Setup sensor
temp_sensor = ADC(Pin(26))

def log_temperature():
    """Log temperature to CSV file"""
    # Read sensor
    raw = temp_sensor.read_u16()
    temp = raw * 3.3 / 65535 * 100  # Convert to Celsius

    # Get timestamp
    timestamp = time.ticks_ms()

    # Append to file
    with open('temp_log.csv', 'a') as f:
        f.write(f'{timestamp},{temp:.1f}\n')

    print(f"Logged: {temp:.1f}°C")

# Main loop
while True:
    log_temperature()
    time.sleep(60)  # Log every minute
```

## Working with JSON (Configuration Files)

JSON is perfect for storing configuration data.

### Saving Configuration

```python
import json

# Configuration dictionary
config = {
    'motor_speed': 150,
    'sensor_threshold': 512,
    'led_brightness': 128,
    'wifi_ssid': 'RobotNet',
    'debug_mode': True
}

# Save to JSON file
with open('config.json', 'w') as f:
    json.dump(config, f)
```

### Loading Configuration

```python
import json

def load_config():
    """Load configuration with defaults fallback"""
    try:
        with open('config.json', 'r') as f:
            config = json.load(f)
        print("Config loaded")
    except OSError:
        print("Config not found, using defaults")
        config = {
            'motor_speed': 100,
            'sensor_threshold': 512,
            'led_brightness': 255,
            'debug_mode': False
        }

        # Save defaults
        with open('config.json', 'w') as f:
            json.dump(config, f)

    return config

# Usage
config = load_config()
motor_speed = config['motor_speed']
print(f"Motor speed: {motor_speed}")
```

### Updating Configuration

```python
def update_config(key, value):
    """Update single configuration value"""
    config = load_config()
    config[key] = value

    with open('config.json', 'w') as f:
        json.dump(config, f)

    print(f"Updated {key} = {value}")

# Usage
update_config('motor_speed', 180)
```

## CSV Data Logging

CSV files are great for sensor data that you'll analyze later.

### Writing CSV Data

```python
import time
from machine import ADC, Pin

sensor1 = ADC(Pin(26))
sensor2 = ADC(Pin(27))

# Create CSV with header
with open('sensor_log.csv', 'w') as f:
    f.write('timestamp,sensor1,sensor2\n')

# Log data
def log_sensors():
    timestamp = time.ticks_ms()
    val1 = sensor1.read_u16()
    val2 = sensor2.read_u16()

    with open('sensor_log.csv', 'a') as f:
        f.write(f'{timestamp},{val1},{val2}\n')

# Log every second for 1 minute
for _ in range(60):
    log_sensors()
    time.sleep(1)

print("Logging complete! 60 samples recorded")
```

### Reading CSV Data

```python
def analyze_log():
    """Read and analyze CSV log file"""
    with open('sensor_log.csv', 'r') as f:
        # Skip header
        next(f)

        values = []
        for line in f:
            # Parse CSV line
            parts = line.strip().split(',')
            timestamp = int(parts[0])
            sensor1 = int(parts[1])
            sensor2 = int(parts[2])

            values.append((timestamp, sensor1, sensor2))

    # Calculate statistics
    sensor1_values = [v[1] for v in values]
    avg = sum(sensor1_values) / len(sensor1_values)
    min_val = min(sensor1_values)
    max_val = max(sensor1_values)

    print(f"Samples: {len(values)}")
    print(f"Sensor1 - Avg: {avg:.1f}, Min: {min_val}, Max: {max_val}")

analyze_log()
```

## File System Management

### Checking Free Space

```python
import os

def check_storage():
    """Check filesystem usage"""
    stat = os.statvfs('/')

    # Calculate sizes in bytes
    block_size = stat[0]
    total_blocks = stat[2]
    free_blocks = stat[3]

    total_size = total_blocks * block_size
    free_size = free_blocks * block_size
    used_size = total_size - free_size

    # Convert to KB
    print(f"Total: {total_size / 1024:.1f} KB")
    print(f"Used: {used_size / 1024:.1f} KB")
    print(f"Free: {free_size / 1024:.1f} KB")
    print(f"Usage: {used_size / total_size * 100:.1f}%")

check_storage()
```

### Rotating Log Files

```python
import os

def rotate_log(filename, max_size=10000):
    """Rotate log file if it exceeds max_size bytes"""
    try:
        size = os.stat(filename)[6]

        if size > max_size:
            # Rename old log
            old_name = filename.replace('.', '_old.')
            try:
                os.remove(old_name)  # Delete old backup
            except OSError:
                pass

            os.rename(filename, old_name)
            print(f"Rotated {filename} to {old_name}")

            return True
    except OSError:
        pass  # File doesn't exist yet

    return False

# Usage
rotate_log('sensor_log.csv', max_size=50000)

# Now log to fresh file
with open('sensor_log.csv', 'a') as f:
    f.write('new data\n')
```

## Binary Files

For efficiency, you can store data in binary format.

```python
import struct

# Write binary data
with open('sensor_data.bin', 'wb') as f:
    # Pack 3 integers into binary (4 bytes each)
    data = struct.pack('III', 1234, 5678, 9012)
    f.write(data)

# Read binary data
with open('sensor_data.bin', 'rb') as f:
    data = f.read()
    # Unpack 3 integers
    values = struct.unpack('III', data)
    print(values)  # (1234, 5678, 9012)
```

**Format codes:**
- `'I'` - Unsigned int (4 bytes)
- `'H'` - Unsigned short (2 bytes)
- `'f'` - Float (4 bytes)
- `'d'` - Double (8 bytes)

Binary files are ~4x more space-efficient than CSV for numeric data.

## Try It Yourself

### Exercise 1: Calibration Storage
Create a calibration system:

```python
def calibrate_sensor(sensor, num_samples=100):
    """Calibrate sensor and save result"""
    # Read sensor multiple times
    # Calculate min, max, average
    # Save to calibration.json
    pass

def apply_calibration(raw_value):
    """Apply saved calibration to raw sensor reading"""
    # Load calibration.json
    # Map raw_value using stored calibration
    pass
```

### Exercise 2: Event Logger
Create an event logging system:

```python
def log_event(event_type, message):
    """
    Log event with timestamp
    Format: [2024-01-20 14:30:45] EVENT_TYPE: Message
    """
    pass

# Usage:
# log_event('INFO', 'Robot started')
# log_event('WARNING', 'Low battery: 3.2V')
# log_event('ERROR', 'Sensor disconnected')
```

### Exercise 3: High-Score Tracker
Create a game high-score system:

```python
def save_score(player_name, score):
    """Save score to highscores.json"""
    pass

def get_top_scores(n=10):
    """Get top N scores"""
    pass

def display_leaderboard():
    """Print formatted leaderboard"""
    pass
```

### Exercise 4: Settings Manager
Create a persistent settings system with UI:

```python
class Settings:
    def __init__(self, filename='settings.json'):
        pass

    def get(self, key, default=None):
        """Get setting with default fallback"""
        pass

    def set(self, key, value):
        """Set setting and save immediately"""
        pass

    def reset_to_defaults(self):
        """Reset all settings to defaults"""
        pass

# Usage:
# settings = Settings()
# speed = settings.get('motor_speed', 100)
# settings.set('motor_speed', 150)
```

## Common Issues

### Issue: File Corruption
**Problem:** Power loss during write can corrupt files.

**Solution:**
```python
# Write to temporary file first
with open('data.tmp', 'w') as f:
    f.write(important_data)

# Then rename (atomic operation)
os.rename('data.tmp', 'data.txt')
```

### Issue: Running Out of Space
**Problem:** Flash fills up with logs.

**Solution:** Implement log rotation and cleanup:
```python
def cleanup_old_logs():
    """Delete old log files"""
    for filename in os.listdir():
        if filename.startswith('log_') and filename.endswith('.csv'):
            # Check if older than 7 days or size > 100KB
            os.remove(filename)
```

### Issue: Slow File Operations
**Problem:** Writing to file in tight loop slows down program.

**Solution:** Buffer writes in memory:
```python
buffer = []

def log_data(value):
    buffer.append(value)

    # Flush every 100 samples
    if len(buffer) >= 100:
        with open('log.csv', 'a') as f:
            for val in buffer:
                f.write(f'{val}\n')
        buffer.clear()
```

## Summary

| Feature | Arduino (No Filesystem) | MicroPython (Built-In) |
|---------|------------------------|------------------------|
| **Built-in storage** | EEPROM only (1-4 KB) | Flash filesystem (1-2 MB) |
| **External storage** | SD card module required | Not needed (but possible) |
| **File operations** | SD library functions | Standard Python `open()` |
| **Text files** | Possible with SD card | Yes, built-in |
| **JSON support** | Limited, need library | Built-in `json` module |
| **CSV logging** | Possible with SD card | Easy with string formatting |
| **Complexity** | High (extra hardware/wiring) | Low (built-in) |

**MicroPython Advantages:**
- No extra hardware needed
- Standard Python file operations
- JSON support built-in
- Easy configuration management
- Simple data logging
- File system navigation

**Use Cases:**
- Sensor data logging
- Configuration storage
- Calibration data
- High scores / persistent state
- Debugging / event logs
- Offline data collection

**Best Practices:**
- Always use `with` statement
- Implement log rotation
- Buffer writes for performance
- Use JSON for configuration
- Use CSV for tabular data
- Use binary for efficient numeric storage
- Handle errors gracefully
- Check free space periodically

You now have a capability that standard Arduino lacks - a built-in file system. This opens up new possibilities for your projects, from data logging robots to configurable systems that remember settings between power cycles.

---

> **Previous Lesson:** [Libraries vs Modules](/learn/arduino_to_python/16_libraries_vs_modules.html) |
> **Next Lesson:** [Understanding Dual Processor](/learn/arduino_to_python/18_understanding_dual_processor.html)
>
> **Course Home:** [Arduino to Python](/learn/arduino_to_python/)
