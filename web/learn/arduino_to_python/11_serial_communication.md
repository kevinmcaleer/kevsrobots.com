---
layout: lesson
title: Serial Communication - Serial vs UART
author: Kevin McAleer
type: page
cover: /learn/arduino_to_python/assets/serial_communication.jpg
date: 2025-01-20
previous: 10_analog_io.html
next: 12_timing_and_delays.html
description: Comparing Arduino Serial class with MicroPython UART for communication
percent: 36
duration: 9
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


![Serial Communication cover image](assets/serial_communication.jpg){:class="cover"}

## Introduction

Serial communication is the backbone of Arduino projects - you use `Serial.print()` for debugging, `Serial.begin()` to start communication, and `Serial.read()` to receive data. It's ubiquitous and straightforward.

MicroPython handles serial communication differently. There's no single "Serial" object. Instead, you work with `UART` (Universal Asynchronous Receiver-Transmitter) objects for hardware serial ports, and `print()` for USB serial debugging.

In this lesson, you'll learn how MicroPython's UART class compares to Arduino's Serial, how to send and receive data, and understand the key differences between USB serial debugging and hardware UART communication.

## USB Serial vs Hardware UART

First, let's clarify two types of serial communication:

**USB Serial (Debug/Monitor):**
- Arduino: `Serial` object (USB connection to computer)
- Python: `print()` and `input()` (Thonny shell/REPL)
- Used for: Debugging, monitoring, user interaction

**Hardware UART (Communication between devices):**
- Arduino: `Serial1`, `Serial2`, etc. on boards with multiple UARTs
- Python: `UART(0)`, `UART(1)`, etc.
- Used for: GPS modules, ESP32 communication, sensor modules

## USB Serial Debugging

### Arduino C++ (Serial.print)
```cpp
void setup() {
  Serial.begin(9600);  // Initialize USB serial
  Serial.println("Robot starting...");
}

void loop() {
  int sensorValue = analogRead(A0);

  // Print different types
  Serial.print("Sensor: ");
  Serial.print(sensorValue);
  Serial.print(" | Voltage: ");
  Serial.println(sensorValue * 5.0 / 1023.0, 2);  // 2 decimal places

  delay(1000);
}
```

### MicroPython (print)
```python
from machine import ADC, Pin
import time

# Create ADC
sensor = ADC(Pin(26))

print("Robot starting...")

while True:
    sensor_value = sensor.read_u16()

    # Print with f-string (much cleaner!)
    voltage = sensor_value * 3.3 / 65535
    print(f"Sensor: {sensor_value} | Voltage: {voltage:.2f}")

    time.sleep(1)
```

**Key difference:** No initialization needed! `print()` works immediately through USB.

## Hardware UART Communication

### Arduino C++ (Serial1)
```cpp
void setup() {
  // USB serial for debugging
  Serial.begin(9600);

  // Hardware UART on pins TX1/RX1
  Serial1.begin(9600);

  Serial.println("UART initialized");
}

void loop() {
  // Send data to connected device
  Serial1.println("HELLO");

  // Check if data available
  if (Serial1.available()) {
    String received = Serial1.readStringUntil('\n');
    Serial.print("Received: ");
    Serial.println(received);
  }

  delay(1000);
}
```

### MicroPython (UART)
```python
from machine import UART, Pin
import time

# Create UART object (UART 0, pins GPIO0/GPIO1 on Pico)
uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))

print("UART initialized")

while True:
    # Send data to connected device
    uart.write("HELLO\n")

    # Check if data available
    if uart.any():
        received = uart.readline()
        # Decode bytes to string
        message = received.decode('utf-8').strip()
        print(f"Received: {message}")

    time.sleep(1)
```

**Key differences:**
- Arduino: `Serial1`, `Serial2`, etc.
- Python: `UART(0)`, `UART(1)`, etc.
- Python: Must decode bytes to string
- Python: Explicit pin assignment possible

## UART Initialization

### Arduino C++
```cpp
void setup() {
  // Simple initialization
  Serial1.begin(9600);

  // With configuration (8 data bits, no parity, 1 stop bit)
  Serial1.begin(9600, SERIAL_8N1);
}
```

### MicroPython (More Options)
```python
from machine import UART, Pin

# Basic initialization
uart = UART(0, baudrate=9600)

# Full configuration
uart = UART(
    0,                    # UART ID
    baudrate=9600,       # Baud rate
    bits=8,              # Data bits (7 or 8)
    parity=None,         # Parity (None, 0=even, 1=odd)
    stop=1,              # Stop bits (1 or 2)
    tx=Pin(0),           # TX pin
    rx=Pin(1)            # RX pin
)

# Check current settings
print(uart)  # Shows all UART parameters
```

## Sending Data

### Arduino C++
```cpp
// Send string
Serial1.print("Hello");
Serial1.println("World");  // Adds newline

// Send number
int value = 42;
Serial1.print(value);

// Send formatted
char buffer[50];
sprintf(buffer, "Value: %d, Temp: %.1f", value, 23.5);
Serial1.println(buffer);

// Send bytes
byte data[] = {0x01, 0x02, 0x03};
Serial1.write(data, 3);
```

### MicroPython
```python
# Send string (must encode to bytes)
uart.write("Hello")
uart.write("World\n")

# Send number (convert to string first)
value = 42
uart.write(str(value))

# Send formatted (f-strings!)
uart.write(f"Value: {value}, Temp: {23.5:.1f}\n")

# Send bytes directly
data = bytes([0x01, 0x02, 0x03])
uart.write(data)

# Or use bytearray
data = bytearray([0x01, 0x02, 0x03])
uart.write(data)
```

## Receiving Data

### Arduino C++ (Blocking and Non-Blocking)
```cpp
void loop() {
  // Check if data available (non-blocking)
  if (Serial1.available()) {
    // Read single byte
    char c = Serial1.read();

    // Read until newline
    String line = Serial1.readStringUntil('\n');

    // Read with timeout (1000ms)
    Serial1.setTimeout(1000);
    String data = Serial1.readString();

    // Read bytes into buffer
    byte buffer[10];
    int bytesRead = Serial1.readBytes(buffer, 10);
  }

  // Blocking read (waits forever)
  // char c = Serial1.read();  // Don't do this in loop!
}
```

### MicroPython
```python
import time

while True:
    # Check if data available (non-blocking)
    if uart.any():
        # Read single byte
        byte = uart.read(1)  # Returns bytes object

        # Read all available bytes
        data = uart.read()

        # Read until newline
        line = uart.readline()  # Returns bytes

        # Decode to string
        if line:
            message = line.decode('utf-8').strip()
            print(f"Received: {message}")

    time.sleep(0.1)
```

**Important:** MicroPython UART methods return `bytes` objects, not strings. You must decode them!

## Real-World Example: GPS Module Communication

### Arduino C++
```cpp
// Reading GPS NMEA sentences
void setup() {
  Serial.begin(9600);   // USB debug
  Serial1.begin(9600);  // GPS module
}

void loop() {
  if (Serial1.available()) {
    String sentence = Serial1.readStringUntil('\n');

    // Parse NMEA sentence
    if (sentence.startsWith("$GPGGA")) {
      Serial.print("GPS: ");
      Serial.println(sentence);

      // Parse coordinates...
      int comma1 = sentence.indexOf(',', 7);
      int comma2 = sentence.indexOf(',', comma1 + 1);
      String latitude = sentence.substring(comma1 + 1, comma2);

      Serial.print("Lat: ");
      Serial.println(latitude);
    }
  }
}
```

### MicroPython
```python
from machine import UART, Pin
import time

# Setup
uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))

while True:
    if uart.any():
        # Read line
        sentence = uart.readline()

        if sentence:
            # Decode to string
            line = sentence.decode('utf-8').strip()

            # Parse NMEA sentence
            if line.startswith('$GPGGA'):
                print(f"GPS: {line}")

                # Parse coordinates (much easier with split!)
                parts = line.split(',')
                if len(parts) > 2:
                    latitude = parts[2]
                    print(f"Lat: {latitude}")

    time.sleep(0.1)
```

## Command Protocol Example

### Arduino C++ (Robot Command Receiver)
```cpp
void setup() {
  Serial1.begin(9600);
  Serial.begin(9600);
}

void loop() {
  if (Serial1.available()) {
    String command = Serial1.readStringUntil('\n');
    command.trim();  // Remove whitespace

    // Parse command: "MOTOR:LEFT,180,CW"
    if (command.startsWith("MOTOR:")) {
      String params = command.substring(6);

      int comma1 = params.indexOf(',');
      int comma2 = params.indexOf(',', comma1 + 1);

      String motor = params.substring(0, comma1);
      int speed = params.substring(comma1 + 1, comma2).toInt();
      String direction = params.substring(comma2 + 1);

      Serial.print("Motor: ");
      Serial.print(motor);
      Serial.print(", Speed: ");
      Serial.print(speed);
      Serial.print(", Dir: ");
      Serial.println(direction);

      // Control motor...
    }
    else if (command == "STOP") {
      Serial.println("Stopping!");
      // Stop motors...
    }
  }
}
```

### MicroPython (Cleaner Parsing)
```python
from machine import UART, Pin
import time

uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))

def parse_command(command):
    """Parse robot command string"""
    command = command.strip()

    if command.startswith("MOTOR:"):
        # Parse: "MOTOR:LEFT,180,CW"
        params = command[6:].split(',')

        motor = params[0]
        speed = int(params[1])
        direction = params[2]

        print(f"Motor: {motor}, Speed: {speed}, Dir: {direction}")

        # Control motor...
        return True

    elif command == "STOP":
        print("Stopping!")
        # Stop motors...
        return True

    return False

while True:
    if uart.any():
        line = uart.readline()
        if line:
            command = line.decode('utf-8')
            parse_command(command)

    time.sleep(0.01)
```

## Buffering and Flow Control

### Arduino C++ (Limited Buffer)
```cpp
// Arduino has ~64 byte serial buffer
// Data can be lost if not read quickly enough

void setup() {
  Serial.begin(9600);

  // Increase buffer (platform-specific, not always available)
  // Serial.setRxBufferSize(256);
}
```

### MicroPython (Configurable Buffer)
```python
# Create UART with larger RX buffer
uart = UART(
    0,
    baudrate=9600,
    tx=Pin(0),
    rx=Pin(1),
    rxbuf=256  # 256 byte receive buffer
)

# Or configure after creation
uart.init(rxbuf=256)
```

## Try It Yourself

### Exercise 1: Echo Server
Create a UART echo server that receives messages and sends them back:

```python
from machine import UART, Pin
import time

uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))

# Read incoming data and echo it back
# Add "ECHO: " prefix to each line
```

### Exercise 2: Command Line Interface
Build a simple CLI that responds to commands:
- `LED ON` - Turn on LED
- `LED OFF` - Turn off LED
- `STATUS` - Print current state
- `HELP` - Show available commands

### Exercise 3: Bidirectional Communication
Connect two microcontrollers with UART. Have them:
- Exchange sensor readings every second
- Acknowledge receipt of data
- Handle communication errors

### Exercise 4: Protocol Design
Design a robust command protocol with:
- Start/end markers
- Checksums for error detection
- ACK/NACK responses
- Implement parser for this protocol

## Common Issues

### Issue: Garbled Data
**Problem:**
```python
uart = UART(0, baudrate=9600)
# Data received is gibberish
```

**Solution:** Baud rate mismatch! Both devices must use the same baud rate:
```python
# Sender and receiver MUST match
uart = UART(0, baudrate=9600)  # Both must be 9600
```

### Issue: Missing Data
**Problem:** Some messages don't get received.

**Solution:** Increase RX buffer and read more frequently:
```python
uart = UART(0, baudrate=9600, rxbuf=512)  # Larger buffer

# Read in tight loop
while True:
    if uart.any():
        data = uart.read()
        # Process immediately
    time.sleep(0.001)  # Very short delay
```

### Issue: Bytes vs Strings
**Problem:**
```python
data = uart.readline()
if data == "HELLO\n":  # Never matches!
    print("Got hello")
```

**Solution:** Decode bytes first:
```python
data = uart.readline()
if data:
    message = data.decode('utf-8')
    if message == "HELLO\n":
        print("Got hello")
```

### Issue: Blocking Reads
**Problem:** Program hangs waiting for data.

**Solution:** Always check `uart.any()` before reading:
```python
# Good - non-blocking
if uart.any():
    data = uart.read()

# Bad - blocks forever if no data
data = uart.read(10)  # Waits for 10 bytes!
```

## Summary

| Feature | Arduino C++ | MicroPython |
|---------|-------------|-------------|
| **USB Serial** | `Serial.begin(9600)` | `print()` (automatic) |
| **Hardware UART** | `Serial1.begin(9600)` | `UART(0, baudrate=9600)` |
| **Send string** | `Serial1.println("text")` | `uart.write("text\n")` |
| **Send bytes** | `Serial1.write(buffer, size)` | `uart.write(bytes([...]))` |
| **Check available** | `Serial1.available()` | `uart.any()` |
| **Read byte** | `Serial1.read()` | `uart.read(1)` |
| **Read line** | `Serial1.readStringUntil('\n')` | `uart.readline()` |
| **Data type** | Returns `int` or `String` | Returns `bytes` (must decode) |
| **Buffer config** | Limited/fixed | `rxbuf=` parameter |

**Python Advantages:**
- Explicit UART object creation
- Configurable buffer sizes
- Pin assignment flexibility
- `print()` for debugging without initialization

**Arduino Advantages:**
- Automatic String conversion
- `Serial.print()` variations (DEC, HEX, BIN)
- More established ecosystem for serial devices

**Best Practices:**
- Always decode bytes in Python: `data.decode('utf-8')`
- Use `uart.any()` before reading to avoid blocking
- Increase buffer size for high-speed data
- Add newlines as message delimiters
- Implement proper error handling

In the next lesson, we'll explore timing and delays - comparing Arduino's `delay()`, `millis()`, and `micros()` with MicroPython's time functions.

---

> **Previous Lesson:** [Analog I/O](/learn/arduino_to_python/10_analog_io.html) |
> **Next Lesson:** [Timing and Delays](/learn/arduino_to_python/12_timing_and_delays.html)
>
> **Course Home:** [Arduino to Python](/learn/arduino_to_python/)
