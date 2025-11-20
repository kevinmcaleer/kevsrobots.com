---
title: Functions - Goodbye setup and loop!
description: Learn how to define and use functions in MicroPython, including the shift away from Arduino's setup() and loop() pattern
layout: lesson
type: page
cover: /learn/arduino_to_python/assets/cover.jpg
---

## Functions: The Python Way

Functions work differently in Python. Instead of declaring return types like in C++, you just use the `def` keyword. And the biggest change? **No more mandatory `setup()` and `loop()` functions!**

MicroPython programs are just regular Python code that runs from top to bottom. You create your own structure.

## Basic Function Syntax

**Arduino C++:**
```cpp
// Return type comes first
int addNumbers(int a, int b) {
    return a + b;
}

void blinkLED(int pin, int times) {
    for (int i = 0; i < times; i++) {
        digitalWrite(pin, HIGH);
        delay(500);
        digitalWrite(pin, LOW);
        delay(500);
    }
}
```

**MicroPython:**
```python
# Use 'def', no return type declaration
def add_numbers(a, b):
    return a + b

def blink_led(pin, times):
    for i in range(times):
        pin.on()
        time.sleep(0.5)
        pin.off()
        time.sleep(0.5)
```

Key differences:
- **`def` keyword** starts the function definition
- **Colon `:` and indentation** define the function body
- **No return type** declared (Python figures it out)
- **No parameter types** declared
- Function names use `snake_case` (Python convention)

## The setup() and loop() Paradigm is Gone

Arduino forces a specific structure. Python doesn't:

**Arduino C++:**
```cpp
// MUST have setup()
void setup() {
    Serial.begin(9600);
    pinMode(13, OUTPUT);
}

// MUST have loop()
void loop() {
    digitalWrite(13, HIGH);
    delay(1000);
    digitalWrite(13, LOW);
    delay(1000);
}
```

**MicroPython:**
```python
# No setup() required - just do initialization
from machine import Pin
import time

led = Pin(25, Pin.OUT)

# No loop() required - create your own infinite loop
while True:
    led.on()
    time.sleep(1)
    led.off()
    time.sleep(1)
```

In MicroPython:
1. Code at the top level runs once (like `setup()`)
2. You create `while True:` loops where you need them (like `loop()`)
3. You have complete control over program structure

## Functions Without Return Values

Functions that don't return anything work the same:

**Arduino C++:**
```cpp
void printTemperature(float temp) {
    Serial.print("Temperature: ");
    Serial.print(temp);
    Serial.println("C");
}
```

**MicroPython:**
```python
def print_temperature(temp):
    print(f"Temperature: {temp}C")
```

No need for `void` - just don't use `return`, or use `return` without a value.

## Functions With Return Values

**Arduino C++:**
```cpp
float celsiusToFahrenheit(float celsius) {
    return (celsius * 9.0 / 5.0) + 32.0;
}

bool isButtonPressed(int pin) {
    return digitalRead(pin) == HIGH;
}
```

**MicroPython:**
```python
def celsius_to_fahrenheit(celsius):
    return (celsius * 9.0 / 5.0) + 32.0

def is_button_pressed(pin):
    return pin.value() == 1
```

Python automatically figures out the return type. You can even return different types from the same function (though this can be confusing).

## Multiple Return Values

Here's something Python does better than C++: returning multiple values easily.

**Arduino C++:**
```cpp
// Awkward - need to pass pointers or use a struct
void getMinMax(int* minVal, int* maxVal) {
    *minVal = 0;
    *maxVal = 100;
}

// Usage
int minimum, maximum;
getMinMax(&minimum, &maximum);
```

**MicroPython:**
```python
# Easy - return a tuple
def get_min_max():
    return 0, 100  # Returns a tuple

# Usage
minimum, maximum = get_min_max()
```

Python returns multiple values as a tuple, which you can unpack directly. Much cleaner!

## Default Parameters

Python makes default parameters much easier:

**Arduino C++:**
```cpp
// Need function overloading for defaults
void blinkLED(int pin) {
    blinkLED(pin, 1);  // Default to 1 blink
}

void blinkLED(int pin, int times) {
    // Actual implementation
}
```

**MicroPython:**
```python
# Built-in default parameters
def blink_led(pin, times=1, delay=0.5):
    for i in range(times):
        pin.on()
        time.sleep(delay)
        pin.off()
        time.sleep(delay)

# All these work:
blink_led(led)                    # times=1, delay=0.5
blink_led(led, 5)                 # times=5, delay=0.5
blink_led(led, 3, 0.2)            # times=3, delay=0.2
blink_led(led, delay=1.0)         # times=1, delay=1.0 (named arg!)
```

Named arguments let you specify parameters in any order!

## Organizing Your Code

Without `setup()` and `loop()`, how do you structure a program? Here's a common pattern:

**MicroPython:**
```python
from machine import Pin
import time

# Constants at the top
LED_PIN = 25
BUTTON_PIN = 14
BLINK_DELAY = 0.5

# Initialize hardware
led = Pin(LED_PIN, Pin.OUT)
button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_UP)

# Define helper functions
def blink_led(times):
    for i in range(times):
        led.on()
        time.sleep(BLINK_DELAY)
        led.off()
        time.sleep(BLINK_DELAY)

def check_button():
    return button.value() == 0  # Active low

# Main program loop
def main():
    while True:
        if check_button():
            blink_led(3)
        time.sleep(0.1)

# Run the program
if __name__ == '__main__':
    main()
```

This structure:
1. Imports at the top
2. Constants defined
3. Hardware initialized
4. Functions defined
5. `main()` function contains the main loop
6. `if __name__ == '__main__':` runs `main()` when script is executed

## The if __name__ == '__main__': Pattern

This Python pattern might look confusing at first:

```python
def main():
    print("Program running")

if __name__ == '__main__':
    main()
```

**What it means:**
- If this file is run directly, execute `main()`
- If this file is imported by another file, don't run `main()`

For MicroPython projects, you can usually skip this and just put your code at the top level. It's more important for larger Python projects.

## Passing Pins to Functions

A common pattern is passing Pin objects to functions:

**MicroPython:**
```python
from machine import Pin
import time

def set_motor_speed(forward_pin, reverse_pin, speed):
    """Control a motor with direction and speed (0-100%)"""
    if speed > 0:
        forward_pin.on()
        reverse_pin.off()
    elif speed < 0:
        forward_pin.off()
        reverse_pin.on()
    else:
        forward_pin.off()
        reverse_pin.off()

# Setup
motor_fwd = Pin(10, Pin.OUT)
motor_rev = Pin(11, Pin.OUT)

# Usage
set_motor_speed(motor_fwd, motor_rev, 75)   # 75% forward
set_motor_speed(motor_fwd, motor_rev, -50)  # 50% reverse
set_motor_speed(motor_fwd, motor_rev, 0)    # Stop
```

## Lambda Functions (Advanced)

Python has anonymous functions called lambdas - quick one-line functions:

```python
# Regular function
def add(a, b):
    return a + b

# Lambda equivalent
add = lambda a, b: a + b

# Useful for simple transformations
temperatures_c = [20, 25, 30]
temperatures_f = list(map(lambda c: c * 9/5 + 32, temperatures_c))
print(temperatures_f)  # [68.0, 77.0, 86.0]
```

You won't need lambdas often in MicroPython, but they're handy for quick operations.

## Complete Example: Sensor Reading System

**Arduino C++:**
```cpp
const int SENSOR_PIN = A0;
const int LED_PIN = 13;
const int THRESHOLD = 500;

void setup() {
    Serial.begin(9600);
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    int sensorValue = readSensor();
    float voltage = convertToVoltage(sensorValue);
    checkThreshold(voltage);
    delay(1000);
}

int readSensor() {
    return analogRead(SENSOR_PIN);
}

float convertToVoltage(int rawValue) {
    return (rawValue / 1023.0) * 5.0;
}

void checkThreshold(float voltage) {
    if (voltage > 2.5) {
        digitalWrite(LED_PIN, HIGH);
        Serial.println("ALERT!");
    } else {
        digitalWrite(LED_PIN, LOW);
    }
    Serial.print("Voltage: ");
    Serial.println(voltage);
}
```

**MicroPython:**
```python
from machine import Pin, ADC
import time

# Constants
SENSOR_PIN = 26
LED_PIN = 25
VOLTAGE_THRESHOLD = 2.5

# Initialize hardware
sensor = ADC(Pin(SENSOR_PIN))
led = Pin(LED_PIN, Pin.OUT)

# Functions
def read_sensor():
    return sensor.read_u16()

def convert_to_voltage(raw_value):
    return (raw_value / 65535) * 3.3  # 16-bit ADC, 3.3V reference

def check_threshold(voltage):
    if voltage > VOLTAGE_THRESHOLD:
        led.on()
        print("ALERT!")
    else:
        led.off()
    print(f"Voltage: {voltage:.2f}V")

# Main loop
def main():
    while True:
        raw_value = read_sensor()
        voltage = convert_to_voltage(raw_value)
        check_threshold(voltage)
        time.sleep(1)

# Run program
main()
```

Notice how the MicroPython version:
- Doesn't need `setup()` and `loop()`
- Has cleaner function syntax
- Uses f-strings for better output formatting
- Organizes code in a clear, readable way

## Try It Yourself

**Exercise 1: Temperature Converter Functions**

Create two functions:
- `celsius_to_fahrenheit(c)` - converts Celsius to Fahrenheit
- `fahrenheit_to_celsius(f)` - converts Fahrenheit to Celsius

Test them with a few values and print the results.

**Answer:**

```python
def celsius_to_fahrenheit(c):
    return (c * 9/5) + 32

def fahrenheit_to_celsius(f):
    return (f - 32) * 5/9

# Test
print(f"25C = {celsius_to_fahrenheit(25)}F")
print(f"77F = {fahrenheit_to_celsius(77)}C")
print(f"0C = {celsius_to_fahrenheit(0)}F")
print(f"32F = {fahrenheit_to_celsius(32)}C")
```

**Exercise 2: LED Blink Patterns**

Create a function `blink_pattern(pin, pattern, delay)` where:
- `pin` is a Pin object
- `pattern` is a list of 1s (on) and 0s (off), like `[1, 0, 1, 0, 1, 1, 0]`
- `delay` is the time for each step

The function should execute the pattern, then turn off the LED.

**Answer:**

```python
from machine import Pin
import time

def blink_pattern(pin, pattern, delay):
    for state in pattern:
        if state == 1:
            pin.on()
        else:
            pin.off()
        time.sleep(delay)
    pin.off()  # Ensure LED is off at end

# Test
led = Pin(25, Pin.OUT)

# SOS pattern: ... --- ...
sos = [1,0,1,0,1,0,0,  # S (short-short-short)
       1,1,0,1,1,0,1,1,0,0,  # O (long-long-long)
       1,0,1,0,1]  # S (short-short-short)

blink_pattern(led, sos, 0.2)
```

**Exercise 3: Multi-Return Function**

Create a function `analyze_sensor(raw_value)` that takes a raw 16-bit ADC value and returns:
- The voltage (0-3.3V)
- A status string: "LOW" (<1V), "MEDIUM" (1-2.5V), or "HIGH" (>2.5V)
- A boolean indicating if it's in the safe range (1-2.5V)

Test it with values: 0, 20000, 40000, 60000

**Answer:**

```python
def analyze_sensor(raw_value):
    voltage = (raw_value / 65535) * 3.3

    if voltage < 1.0:
        status = "LOW"
    elif voltage <= 2.5:
        status = "MEDIUM"
    else:
        status = "HIGH"

    is_safe = 1.0 <= voltage <= 2.5

    return voltage, status, is_safe

# Test
test_values = [0, 20000, 40000, 60000]

for value in test_values:
    voltage, status, safe = analyze_sensor(value)
    safe_text = "SAFE" if safe else "UNSAFE"
    print(f"Raw: {value:5d} | {voltage:.2f}V | {status:6s} | {safe_text}")
```
