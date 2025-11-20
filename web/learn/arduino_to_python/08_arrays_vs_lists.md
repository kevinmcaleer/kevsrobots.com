---
layout: lesson
title: Fixed Arrays vs Dynamic Lists
author: Kevin McAleer
type: page
cover: /learn/arduino_to_python/assets/arrays_vs_lists.jpg
date: 2025-01-20
previous: 07_strings.html
next: 09_pin_control.html
description: Understanding Python's flexible list data structure compared to C++ arrays
percent: 27
duration: 12
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


![Arrays vs Lists cover image](assets/arrays_vs_lists.jpg){:class="cover"}

## Introduction

In Arduino C++, arrays are fixed-size containers that you declare with a specific length. Want to add an element? Too bad - you need to create a new, larger array and copy everything over.

Python lists are completely different. They grow and shrink dynamically, support mixed types, and come with powerful methods for adding, removing, sorting, and manipulating data. They're one of Python's most beloved features.

In this lesson, you'll learn how Python's lists compare to Arduino arrays, discover list comprehensions (a Python superpower), and understand when to use lists vs tuples.

## Array Declaration: Fixed vs Dynamic

### Arduino C++
```cpp
// Must declare size at compile time
int sensors[5];  // Array of 5 integers

// Initialize with values
int readings[] = {100, 200, 150, 300, 250};  // Size inferred (5)

// Access elements
int first = readings[0];   // 100
readings[2] = 175;         // Modify element

// Size is fixed forever!
// Can't add a 6th element
```

**Arduino array limitations:**
- Size fixed at declaration
- Can't grow or shrink
- No built-in way to get length (must track manually or use sizeof)
- All elements must be same type

### Python
```python
# Create list (dynamic size)
sensors = [100, 200, 150, 300, 250]

# Access elements (same syntax as C++)
first = sensors[0]   # 100
sensors[2] = 175     # Modify element

# But now you can modify the size!
sensors.append(350)  # Add element → [100, 200, 175, 300, 250, 350]
sensors.pop()        # Remove last → [100, 200, 175, 300, 250]

# Get length easily
length = len(sensors)  # 5

# Can mix types (though not recommended for data)
mixed = [100, "sensor", 3.14, True]  # Valid but unusual
```

**Python list advantages:**
- Dynamic sizing
- Built-in methods for manipulation
- Easy length checking
- Can contain mixed types

## Adding and Removing Elements

### Arduino C++ (Manual and Painful)
```cpp
// To "add" element, must copy to new array
int oldArray[5] = {1, 2, 3, 4, 5};
int newArray[6];

// Copy old elements
for (int i = 0; i < 5; i++) {
  newArray[i] = oldArray[i];
}

// Add new element
newArray[5] = 6;

// Now using newArray, old one is useless
// Very tedious!
```

### Python (Built-in Methods)
```python
# Start with list
data = [1, 2, 3, 4, 5]

# Add to end
data.append(6)  # [1, 2, 3, 4, 5, 6]

# Add at specific position
data.insert(0, 0)  # [0, 1, 2, 3, 4, 5, 6]

# Remove by value
data.remove(3)  # [0, 1, 2, 4, 5, 6]

# Remove by index
deleted = data.pop(2)  # Returns 2, list is now [0, 1, 4, 5, 6]

# Remove last element
last = data.pop()  # Returns 6, list is now [0, 1, 4, 5]

# Clear all elements
data.clear()  # []
```

## Real-World Example: Sensor History Buffer

### Arduino C++ (Circular Buffer - Complex)
```cpp
const int BUFFER_SIZE = 10;
int sensorBuffer[BUFFER_SIZE];
int bufferIndex = 0;
int bufferCount = 0;

void addReading(int value) {
  sensorBuffer[bufferIndex] = value;
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;  // Wrap around

  if (bufferCount < BUFFER_SIZE) {
    bufferCount++;
  }
}

float getAverage() {
  if (bufferCount == 0) return 0;

  long sum = 0;
  for (int i = 0; i < bufferCount; i++) {
    sum += sensorBuffer[i];
  }
  return (float)sum / bufferCount;
}
```

### Python (Simple and Clear)
```python
class SensorHistory:
    def __init__(self, max_size=10):
        self.buffer = []
        self.max_size = max_size

    def add_reading(self, value):
        self.buffer.append(value)

        # Keep only last max_size elements
        if len(self.buffer) > self.max_size:
            self.buffer.pop(0)  # Remove oldest

    def get_average(self):
        if not self.buffer:
            return 0
        return sum(self.buffer) / len(self.buffer)

# Usage
history = SensorHistory(10)
history.add_reading(100)
history.add_reading(105)
history.add_reading(102)
print(history.get_average())  # 102.33...
```

**Python benefits:**
- No manual index tracking
- `pop(0)` removes first element easily
- `sum()` built-in for totaling
- Much more readable

## List Slicing: Powerful and Intuitive

### Arduino C++ (Manual Copying)
```cpp
int original[] = {10, 20, 30, 40, 50};

// Get elements 1-3 (indices 1, 2, 3)
int subset[3];
for (int i = 0; i < 3; i++) {
  subset[i] = original[i + 1];
}
// subset is {20, 30, 40}
```

### Python (Slice Notation)
```python
original = [10, 20, 30, 40, 50]

# Get elements 1-3 (indices 1, 2, 3)
subset = original[1:4]  # [20, 30, 40]

# Get first 3 elements
first_three = original[:3]  # [10, 20, 30]

# Get last 2 elements
last_two = original[-2:]  # [40, 50]

# Get every other element
every_other = original[::2]  # [10, 30, 50]

# Reverse list
reversed_list = original[::-1]  # [50, 40, 30, 20, 10]

# Copy entire list
copy = original[:]  # New list with same elements
```

**Slicing syntax:** `list[start:end:step]`
- `start`: First index to include (default 0)
- `end`: First index to exclude (default len(list))
- `step`: Increment (default 1)

## List Methods: Rich Functionality

### Arduino C++ (Very Limited)
```cpp
int data[] = {3, 1, 4, 1, 5, 9, 2, 6};
int size = 8;

// Find maximum (manual loop)
int maxVal = data[0];
for (int i = 1; i < size; i++) {
  if (data[i] > maxVal) {
    maxVal = data[i];
  }
}

// Count occurrences (manual loop)
int count = 0;
int target = 1;
for (int i = 0; i < size; i++) {
  if (data[i] == target) {
    count++;
  }
}
```

### Python (Built-in Methods)
```python
data = [3, 1, 4, 1, 5, 9, 2, 6]

# Find maximum/minimum
max_val = max(data)  # 9
min_val = min(data)  # 1

# Count occurrences
count = data.count(1)  # 2

# Find index of value
index = data.index(4)  # 2

# Check if value exists
if 5 in data:
    print("5 is in the list")

# Sort (modifies in place)
data.sort()  # [1, 1, 2, 3, 4, 5, 6, 9]

# Sort (returns new list, keeps original)
data = [3, 1, 4, 1, 5, 9, 2, 6]
sorted_data = sorted(data)  # [1, 1, 2, 3, 4, 5, 6, 9]
# data unchanged: [3, 1, 4, 1, 5, 9, 2, 6]

# Reverse
data.reverse()  # Modifies in place

# Extend (add another list)
data.extend([10, 11, 12])
```

## List Comprehensions: Python's Secret Weapon

List comprehensions are a concise way to create lists. They have no Arduino equivalent and are considered very "Pythonic."

### Arduino C++ Way
```cpp
// Create array of squares
int numbers[] = {1, 2, 3, 4, 5};
int squares[5];
for (int i = 0; i < 5; i++) {
  squares[i] = numbers[i] * numbers[i];
}
// squares: {1, 4, 9, 16, 25}

// Filter even numbers
int evens[10];  // Don't know final size!
int evenCount = 0;
for (int i = 0; i < 5; i++) {
  if (numbers[i] % 2 == 0) {
    evens[evenCount++] = numbers[i];
  }
}
```

### Python List Comprehensions
```python
# Create list of squares
numbers = [1, 2, 3, 4, 5]
squares = [x * x for x in numbers]  # [1, 4, 9, 16, 25]

# Filter even numbers
evens = [x for x in numbers if x % 2 == 0]  # [2, 4]

# Combine mapping and filtering
even_squares = [x * x for x in numbers if x % 2 == 0]  # [4, 16]

# More complex transformations
sensor_voltages = [0, 512, 1023, 768, 256]
voltages = [val * 3.3 / 1023 for val in sensor_voltages]
# [0.0, 1.65, 3.3, 2.475, 0.825]
```

**List comprehension syntax:**
```python
[expression for item in iterable if condition]
```

## Real-World Robot Example: Multi-Sensor Processing

### Arduino C++ (Verbose)
```cpp
// Read 4 distance sensors, convert to cm, filter outliers
const int NUM_SENSORS = 4;
int sensorPins[] = {A0, A1, A2, A3};
float validDistances[NUM_SENSORS];
int validCount = 0;

void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int raw = analogRead(sensorPins[i]);
    float distance = raw * 0.3;  // Convert to cm

    // Filter outliers (0-100 cm range)
    if (distance >= 0 && distance <= 100) {
      validDistances[validCount++] = distance;
    }
  }
}

float getAverageDistance() {
  if (validCount == 0) return 0;

  float sum = 0;
  for (int i = 0; i < validCount; i++) {
    sum += validDistances[i];
  }
  return sum / validCount;
}
```

### Python (Concise)
```python
from machine import ADC

# Read 4 distance sensors, convert to cm, filter outliers
sensor_pins = [ADC(26), ADC(27), ADC(28), ADC(29)]

def read_sensors():
    # Read all sensors
    raw_values = [adc.read_u16() for adc in sensor_pins]

    # Convert to cm and filter in one step
    distances = [val * 0.3 for val in raw_values if 0 <= val * 0.3 <= 100]

    return distances

def get_average_distance():
    distances = read_sensors()
    return sum(distances) / len(distances) if distances else 0

# Usage
avg = get_average_distance()
print(f"Average distance: {avg:.1f} cm")
```

## Tuples: Immutable Lists

Python has another sequence type: **tuples**. They're like lists but immutable (can't be changed after creation).

### Creating Tuples
```python
# With parentheses
coordinates = (10, 20)

# Without (tuple packing)
rgb = 255, 0, 128

# Single element (need comma!)
single = (42,)

# Convert list to tuple
my_tuple = tuple([1, 2, 3])
```

### When to Use Tuples vs Lists

**Use tuples for:**
- Fixed data (coordinates, RGB values, pin configurations)
- Function returns of multiple values
- Dictionary keys (lists can't be keys)
- Data that shouldn't change

**Use lists for:**
- Collections that grow/shrink
- Sensor readings over time
- Data you'll modify

### Example: Robot Configuration
```python
# Pin configuration (shouldn't change) - use tuple
MOTOR_PINS = (9, 10, 11, 12)
RGB_LED_PINS = (6, 7, 8)

# Sensor readings (will change) - use list
sensor_history = []

def add_reading(value):
    sensor_history.append(value)

# Multiple return values
def get_position():
    x, y, heading = read_gps_and_compass()
    return x, y, heading  # Returns tuple

# Unpacking
x, y, heading = get_position()
```

## Nested Lists: 2D Arrays and More

### Arduino C++ (2D Arrays)
```cpp
// 2D array (3 rows, 4 columns)
int matrix[3][4] = {
  {1, 2, 3, 4},
  {5, 6, 7, 8},
  {9, 10, 11, 12}
};

// Access element
int value = matrix[1][2];  // 7

// Must track dimensions manually
```

### Python (Nested Lists)
```python
# 2D list (3 rows, 4 columns)
matrix = [
    [1, 2, 3, 4],
    [5, 6, 7, 8],
    [9, 10, 11, 12]
]

# Access element
value = matrix[1][2]  # 7

# Get dimensions
num_rows = len(matrix)       # 3
num_cols = len(matrix[0])    # 4

# Create with comprehension
zeros = [[0 for _ in range(4)] for _ in range(3)]
# [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]

# Practical example: LED matrix
led_matrix = [[False] * 8 for _ in range(8)]  # 8x8 off LEDs
led_matrix[3][3] = True  # Turn on center LED
```

## Try It Yourself

### Exercise 1: Moving Average Filter
Implement a moving average filter that keeps the last 5 sensor readings:

```python
class MovingAverage:
    def __init__(self, window_size=5):
        # Your code here
        pass

    def add(self, value):
        # Add new value, maintain window size
        pass

    def get_average(self):
        # Return current average
        pass

# Test
ma = MovingAverage(5)
for reading in [100, 105, 102, 98, 101, 103]:
    ma.add(reading)
    print(f"Average: {ma.get_average():.1f}")
```

### Exercise 2: Sensor Calibration
Given a list of raw sensor readings, normalize them to 0-100 range:

```python
raw_readings = [245, 389, 512, 678, 823, 956]
# Convert to 0-100 range (assuming raw range is 0-1023)
normalized = # Your list comprehension here
```

### Exercise 3: Find Closest Obstacle
Given a list of distance sensor readings from different angles, find the closest obstacle:

```python
sensors = [
    (0, 45),    # (angle, distance)
    (45, 67),
    (90, 23),
    (135, 89),
    (180, 34)
]

# Find angle with minimum distance
# Your code here
```

### Exercise 4: Data Filtering
Remove outliers (values outside 2 standard deviations) from sensor data:

```python
import math

data = [100, 102, 105, 98, 101, 999, 103, 97, 104, -50, 99]

# Calculate mean
mean = sum(data) / len(data)

# Calculate standard deviation
variance = sum((x - mean) ** 2 for x in data) / len(data)
std_dev = math.sqrt(variance)

# Filter outliers (|value - mean| < 2 * std_dev)
filtered = # Your list comprehension here
```

## Common Issues

### Issue: Modifying List While Iterating
**Problem:**
```python
data = [1, 2, 3, 4, 5]
for item in data:
    if item % 2 == 0:
        data.remove(item)  # Dangerous! Skips elements
```

**Solution:** Use list comprehension or iterate over copy:
```python
# Best: List comprehension
data = [item for item in data if item % 2 != 0]

# Or iterate over copy
for item in data[:]:
    if item % 2 == 0:
        data.remove(item)
```

### Issue: Shallow Copy vs Deep Copy
**Problem:**
```python
original = [[1, 2], [3, 4]]
copy = original  # Not a copy! Same list
copy[0][0] = 99
print(original)  # [[99, 2], [3, 4]] - modified!
```

**Solution:**
```python
# Shallow copy (for simple lists)
copy = original[:]
copy = original.copy()
copy = list(original)

# Deep copy (for nested structures)
import copy
deep_copy = copy.deepcopy(original)
```

## Summary

| Feature | Arduino C++ | Python |
|---------|-------------|---------|
| **Declaration** | `int arr[5]` | `lst = [1, 2, 3]` |
| **Size** | Fixed | Dynamic |
| **Get length** | `sizeof(arr)/sizeof(arr[0])` | `len(lst)` |
| **Add element** | Create new array | `lst.append(val)` |
| **Remove element** | Manual shift | `lst.remove(val)` or `lst.pop()` |
| **Sort** | Manual or qsort() | `lst.sort()` or `sorted(lst)` |
| **Find value** | Manual loop | `val in lst` or `lst.index(val)` |
| **Slice** | Manual copy | `lst[start:end]` |
| **2D arrays** | `int arr[3][4]` | `lst = [[...], [...]]` |
| **Comprehensions** | No equivalent | `[x*2 for x in lst]` |

**Python Lists Win For:**
- Dynamic sizing needs
- Frequent additions/removals
- Complex data manipulations
- Built-in operations (sort, search, etc.)
- Rapid prototyping

**Use Tuples When:**
- Data shouldn't change
- Returning multiple values
- Dictionary keys
- Slight performance benefit needed

In the next lesson, we'll explore digital pin control - comparing Arduino's `pinMode()` and `digitalWrite()` with MicroPython's object-oriented `Pin` class.

---

> **Previous Lesson:** [Strings](/learn/arduino_to_python/07_strings.html) |
> **Next Lesson:** [Pin Control](/learn/arduino_to_python/09_pin_control.html)
>
> **Course Home:** [Arduino to Python](/learn/arduino_to_python/)
