---
layout: lesson
title: Memory Management - Manual vs Automatic
author: Kevin McAleer
type: page
cover: /learn/arduino_to_python/assets/memory_management.jpg
date: 2025-01-20
previous: 13_interrupts.html
next: 15_object_oriented.html
description: Understanding how Python's garbage collection compares to C++ manual
  memory management
percent: 45
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


![Memory Management cover image](assets/memory_management.jpg){:class="cover"}

## Introduction

Memory management is one of the most fundamental differences between C++ and Python. In Arduino C++, you're acutely aware of memory - you calculate array sizes, worry about stack overflow, and debug mysterious crashes caused by running out of RAM.

Python handles memory automatically through garbage collection. You create objects, use them, and Python cleans them up when done. It sounds like magic, but understanding how it works - and its costs - is crucial for embedded systems with limited RAM.

In this lesson, you'll learn how Python's automatic memory management compares to C++'s manual approach, understand the memory overhead, and discover strategies for efficient memory use in MicroPython.

## The Memory Landscape

### Arduino Memory (ATmega328P - Arduino Uno)
```
┌─────────────────────────────────────┐
│  32 KB Flash (program storage)     │  ← Your code
├─────────────────────────────────────┤
│  2 KB SRAM (dynamic memory)        │  ← Variables, stack, heap
├─────────────────────────────────────┤
│  1 KB EEPROM (persistent storage)   │  ← Saved data
└─────────────────────────────────────┘
```

### Raspberry Pi Pico Memory
```
┌─────────────────────────────────────┐
│  2 MB Flash (program storage)      │  ← Your code + MicroPython
├─────────────────────────────────────┤
│  264 KB SRAM (dynamic memory)      │  ← Variables, heap, stack
└─────────────────────────────────────┘
```

**Key difference:** MicroPython needs 100-200 KB for itself, leaving you with ~100-150 KB for your program.

## Stack vs Heap

### Arduino C++ (Manual Control)
```cpp
void loop() {
  // Stack allocation - automatic, fast, limited
  int localVar = 42;        // On stack
  int array[10];            // On stack
  // Automatically freed when function exits

  // Heap allocation - manual, flexible, dangerous
  int* dynamicArray = (int*)malloc(100 * sizeof(int));
  if (dynamicArray == NULL) {
    // Out of memory!
    Serial.println("Allocation failed!");
    return;
  }

  // Use array...

  // MUST manually free!
  free(dynamicArray);  // Memory leak if you forget this!
}
```

**C++ memory rules:**
- Stack: Automatic, fast, limited size (a few KB)
- Heap: Manual, flexible, larger, but you must `malloc()` and `free()`
- Forgetting `free()` causes memory leaks
- Using freed memory causes crashes

### Python (Automatic Garbage Collection)
```python
def my_function():
    # Everything is on the heap, but you don't manage it
    local_var = 42             # Python allocates
    array = [0] * 10           # Python allocates
    large_list = [0] * 1000    # Python allocates

    # Use variables...

    # Python automatically frees memory when function exits
    # No manual free() needed!

# After function returns, Python's garbage collector
# will eventually free the memory
```

**Python memory rules:**
- Everything is heap-allocated (even integers!)
- Automatic garbage collection
- You never call `free()`
- Higher memory overhead
- Garbage collection has performance cost

## Memory Overhead: The Python Price

### Arduino C++ (Minimal Overhead)
```cpp
int value = 42;  // 2 bytes (Arduino Uno)
// Memory: 2 bytes total
```

### Python (Object Overhead)
```python
value = 42  # Integer object
# Memory: ~28 bytes (object header + value + reference count)
```

**Why so much more?**
- Object header: Type information, reference count
- Dynamic typing: Type must be stored with value
- Garbage collection: Bookkeeping data

### Array Comparison

**Arduino C++:**
```cpp
int readings[100];  // 200 bytes (2 bytes × 100)
```

**Python:**
```python
readings = [0] * 100  # ~3.2 KB!
# Each integer is an object (28 bytes × 100 = 2.8 KB)
# Plus list overhead (400+ bytes)
```

**Python's cost:** ~16x more memory for the same data!

## String Memory: A Cautionary Tale

### Arduino C++ (Memory Fragmentation Hell)
```cpp
void loop() {
  String message = "Sensor: ";
  message += String(analogRead(A0));
  message += " - ";
  message += String(millis());
  message += " ms";

  Serial.println(message);

  // After ~10 minutes of running, program crashes!
  // Why? String concatenation fragments heap memory
  // Eventually, no contiguous block big enough for allocation
}
```

**The Arduino String Problem:**
- Each `+=` allocates new memory
- Old memory is freed
- Heap becomes fragmented (Swiss cheese memory)
- Eventually, allocation fails even though total free memory exists

**Arduino C++ Solution: Use char arrays**
```cpp
void loop() {
  char message[50];
  snprintf(message, 50, "Sensor: %d - %lu ms",
           analogRead(A0), millis());
  Serial.println(message);

  // No dynamic allocation, no fragmentation!
}
```

### Python (Garbage Collection Handles It)
```python
while True:
    message = "Sensor: "
    message += str(sensor.read_u16())
    message += " - "
    message += str(time.ticks_ms())
    message += " ms"

    print(message)

    time.sleep(1)

# Runs forever! Garbage collector prevents fragmentation
```

**Why Python is better here:**
- Strings are immutable (old ones are discarded)
- Garbage collector compacts memory
- No manual management needed
- Overhead: Uses more memory, but doesn't leak or fragment

## Garbage Collection: How It Works

Python uses reference counting with cycle detection:

```python
# Create object (reference count = 1)
data = [1, 2, 3, 4, 5]

# Create another reference (reference count = 2)
backup = data

# Delete one reference (reference count = 1)
del backup

# Delete last reference (reference count = 0)
del data
# Garbage collector frees memory (eventually)
```

**When does garbage collection run?**
- Automatically when memory is low
- When reference count reaches zero
- Periodically based on allocation rate

**You can manually trigger it:**
```python
import gc

# Trigger garbage collection now
gc.collect()

# Check free memory
print(gc.mem_free())

# Disable automatic GC (advanced!)
gc.disable()

# Re-enable
gc.enable()
```

## Real-World Example: Sensor Data Logging

### Arduino C++ (Careful Memory Management)
```cpp
const int MAX_READINGS = 100;
int readings[MAX_READINGS];  // Fixed size
int readingCount = 0;

void loop() {
  int value = analogRead(A0);

  // Only store if space available
  if (readingCount < MAX_READINGS) {
    readings[readingCount++] = value;
  } else {
    // Shift array to make room (expensive!)
    for (int i = 0; i < MAX_READINGS - 1; i++) {
      readings[i] = readings[i + 1];
    }
    readings[MAX_READINGS - 1] = value;
  }

  delay(100);
}

// Memory: 200 bytes, never changes
```

### Python (Dynamic with GC)
```python
from machine import ADC, Pin
import time

sensor = ADC(Pin(26))
readings = []
MAX_READINGS = 100

while True:
    value = sensor.read_u16()

    # Append to list
    readings.append(value)

    # Keep only last MAX_READINGS
    if len(readings) > MAX_READINGS:
        readings.pop(0)  # Remove oldest

    time.sleep(0.1)

# Memory: Grows dynamically, old objects garbage collected
# Memory usage: ~3.2 KB (100 integers)
```

## Memory-Efficient Python Techniques

### 1. Use bytearrays for Binary Data

**Instead of lists:**
```python
# List of bytes - inefficient
data = [0x01, 0x02, 0x03, 0x04]  # ~112 bytes

# Bytearray - efficient
data = bytearray([0x01, 0x02, 0x03, 0x04])  # ~40 bytes
```

### 2. Use array module for Numeric Data

```python
import array

# List of integers - ~2.8 KB for 100 items
readings = [0] * 100

# array of integers - ~200 bytes for 100 items (16-bit)
readings = array.array('H', [0] * 100)  # 'H' = unsigned short (16-bit)
```

**Array type codes:**
- `'b'`: signed byte (8-bit)
- `'B'`: unsigned byte (8-bit)
- `'h'`: signed short (16-bit)
- `'H'`: unsigned short (16-bit)
- `'i'`: signed int (32-bit)
- `'I'`: unsigned int (32-bit)
- `'f'`: float (32-bit)

### 3. Reuse Objects Instead of Creating New Ones

**Inefficient:**
```python
while True:
    message = f"Sensor: {sensor.read()}"  # Creates new string each time
    print(message)
    time.sleep(1)
```

**More efficient:**
```python
# Pre-allocate buffer
message = "Sensor: 0000"

while True:
    value = sensor.read()
    # Reuse buffer (more complex but saves memory)
    print(f"Sensor: {value}")
    time.sleep(1)
```

### 4. Manually Trigger Garbage Collection

```python
import gc

# At startup
gc.collect()  # Free any leftover memory from boot

while True:
    # Do work...
    process_data()

    # Periodically collect garbage
    if time.ticks_ms() % 10000 == 0:
        gc.collect()
```

## Monitoring Memory Usage

### Check Available Memory

```python
import gc

print(f"Free memory: {gc.mem_free()} bytes")
print(f"Allocated memory: {gc.mem_alloc()} bytes")

# Before operation
start_free = gc.mem_free()

# Do something
big_list = [0] * 1000

# After operation
end_free = gc.mem_free()
print(f"Memory used: {start_free - end_free} bytes")
```

### Memory Profiling Example

```python
import gc
import time

def profile_memory(func):
    """Decorator to profile memory usage"""
    def wrapper(*args, **kwargs):
        gc.collect()  # Clean up first
        start = gc.mem_free()

        result = func(*args, **kwargs)

        end = gc.mem_free()
        print(f"{func.__name__} used {start - end} bytes")

        return result
    return wrapper

@profile_memory
def create_large_list():
    return [0] * 1000

@profile_memory
def create_bytearray():
    return bytearray(1000)

# Test
create_large_list()   # Uses ~28 KB
create_bytearray()    # Uses ~1 KB
```

## Common Memory Issues

### Issue 1: MemoryError
**Problem:**
```python
# Allocate too much memory
huge_list = [0] * 100000  # MemoryError!
```

**Solution:**
- Reduce data size
- Use more efficient data structures (array, bytearray)
- Manually run `gc.collect()` before large allocations

### Issue 2: Slow Performance from GC
**Problem:** Program pauses unpredictably.

**Solution:**
```python
import gc

# Disable automatic GC
gc.disable()

# Manually run GC at convenient times
while True:
    # Time-critical code
    read_sensors()
    control_motors()

    # Non-critical time - trigger GC
    gc.collect()
    time.sleep(0.01)
```

### Issue 3: Memory Leak (Python can leak too!)
**Problem:**
```python
# Global list keeps growing
sensor_history = []

while True:
    sensor_history.append(sensor.read())
    # Never removes old data - eventually runs out of memory!
```

**Solution:**
```python
MAX_HISTORY = 1000

while True:
    sensor_history.append(sensor.read())

    # Limit size
    if len(sensor_history) > MAX_HISTORY:
        sensor_history.pop(0)
```

## Try It Yourself

### Exercise 1: Memory Profiling
Compare memory usage of different data structures:
```python
import gc

# Test 1: List of integers
gc.collect()
mem_start = gc.mem_free()
data = [0] * 100
mem_end = gc.mem_free()
print(f"List: {mem_start - mem_end} bytes")

# Test 2: bytearray
gc.collect()
mem_start = gc.mem_free()
data = bytearray(100)
mem_end = gc.mem_free()
print(f"Bytearray: {mem_start - mem_end} bytes")

# Test 3: array.array
import array
gc.collect()
mem_start = gc.mem_free()
data = array.array('H', [0] * 100)
mem_end = gc.mem_free()
print(f"Array: {mem_start - mem_end} bytes")
```

### Exercise 2: Circular Buffer
Implement a memory-efficient circular buffer:
```python
import array

class CircularBuffer:
    def __init__(self, size):
        self.buffer = array.array('H', [0] * size)
        self.index = 0
        self.size = size

    def add(self, value):
        # Add value and wrap around
        pass

    def get_average(self):
        # Calculate average
        pass

# Test - should use constant memory
cb = CircularBuffer(100)
for i in range(10000):
    cb.add(i % 1000)
    if i % 100 == 0:
        print(f"Free memory: {gc.mem_free()}")
```

### Exercise 3: Memory Monitor
Create a continuous memory monitor:
```python
# Display free memory every second
# Alert if memory drops below threshold
# Trigger GC if memory low
```

## Summary

| Aspect | Arduino C++ | Python |
|--------|-------------|---------|
| **Allocation** | Manual (`malloc()`) | Automatic |
| **Deallocation** | Manual (`free()`) | Garbage collection |
| **Memory per int** | 2-4 bytes | ~28 bytes |
| **Memory per array** | Fixed, minimal | Dynamic, overhead |
| **Fragmentation** | Common problem | GC prevents |
| **Control** | Full control | Limited control |
| **Ease of use** | Complex, error-prone | Simple, automatic |
| **Overhead** | Minimal | Significant |

**Python Advantages:**
- No manual memory management
- No memory leaks (usually)
- No use-after-free bugs
- Garbage collection prevents fragmentation
- Simpler code

**C++ Advantages:**
- Minimal memory overhead
- Predictable performance
- More memory available for application
- Full control over allocation timing
- No GC pauses

**Best Practices for MicroPython:**
1. Use `array.array()` or `bytearray()` for large data
2. Limit growth of global lists
3. Manually call `gc.collect()` at convenient times
4. Monitor memory usage during development
5. Be aware of object overhead
6. Reuse objects when possible

**When Memory Really Matters:**
- Consider C++ for memory-constrained projects
- Use MicroPython for rapid development
- Hybrid approach: C++ for critical parts, Python for the rest (Arduino Uno R4)

Congratulations! You now understand the fundamental differences between C++ and Python memory management. This knowledge will help you write efficient embedded Python code and choose the right language for your projects.

---

> **Previous Lesson:** [Interrupts](/learn/arduino_to_python/13_interrupts.html) |
> **Next Lesson:** [Object-Oriented Programming](/learn/arduino_to_python/15_object_oriented.html)
>
> **Course Home:** [Arduino to Python](/learn/arduino_to_python/)
