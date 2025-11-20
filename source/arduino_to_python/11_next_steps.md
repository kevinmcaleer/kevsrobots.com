---
title: Next Steps and Quick Reference
description: Course summary, quick reference guide for converting Arduino C++ to MicroPython, and resources for continuing your learning journey
layout: lesson
type: page
cover: /learn/arduino_to_python/assets/cover.jpg
---

## Congratulations!

You've completed the Arduino to MicroPython course! You now understand the key differences between Arduino C++ and MicroPython, and you've built two complete projects demonstrating IoT and robotics applications.

Let's review what you've learned and where to go next.

## What You've Learned

**Core Concepts:**
- Python syntax vs C++ syntax (no semicolons, indentation matters)
- Variables and types (dynamic typing)
- Control flow (if/elif/else, for/while loops)
- Functions (def keyword, no return types)
- Classes and object-oriented programming

**Hardware Control:**
- Digital pin control (Pin class)
- Analog reading (ADC class)
- PWM output (PWM class)
- Timing and delays (time module)

**Communication:**
- Serial output with print()
- Interactive REPL debugging
- WiFi networking
- Web servers

**Projects:**
- IoT temperature monitor with web interface
- Robot with obstacle avoidance and motor control

## Quick Reference Guide

### Arduino C++ to MicroPython Conversion

**Basic Syntax:**

| Arduino C++ | MicroPython | Notes |
|-------------|-------------|-------|
| `int x = 10;` | `x = 10` | No type declaration, no semicolon |
| `float voltage = 3.3;` | `voltage = 3.3` | Type inferred automatically |
| `String msg = "Hello";` | `msg = "Hello"` | Use `str` type |
| `bool flag = true;` | `flag = True` | Capital T! |
| `// comment` | `# comment` | Different comment symbol |
{:class="table table-single"}

**Control Flow:**

| Arduino C++ | MicroPython | Notes |
|-------------|-------------|-------|
| `if (x > 10) { }` | `if x > 10:` | Colon and indentation |
| `else if (x > 5) { }` | `elif x > 5:` | Use `elif` |
| `else { }` | `else:` | Colon and indentation |
| `for (int i=0; i<10; i++)` | `for i in range(10):` | Use range() |
| `while (x < 100) { }` | `while x < 100:` | Colon and indentation |
| `&&` | `and` | Logical AND |
| `||` | `or` | Logical OR |
| `!` | `not` | Logical NOT |
{:class="table table-single"}

**Functions:**

| Arduino C++ | MicroPython | Notes |
|-------------|-------------|-------|
| `void setup() { }` | Not required | Just write code at top level |
| `void loop() { }` | `while True:` | Explicit infinite loop |
| `int addNumbers(int a, int b)` | `def add_numbers(a, b):` | No types, use `def` |
| `return value;` | `return value` | No semicolon |
{:class="table table-single"}

**Digital I/O:**

| Arduino C++ | MicroPython | Notes |
|-------------|-------------|-------|
| `pinMode(13, OUTPUT)` | `led = Pin(13, Pin.OUT)` | Create Pin object |
| `pinMode(2, INPUT_PULLUP)` | `btn = Pin(2, Pin.IN, Pin.PULL_UP)` | Pin object with pull-up |
| `digitalWrite(13, HIGH)` | `led.on()` or `led.value(1)` | Call method on Pin |
| `digitalWrite(13, LOW)` | `led.off()` or `led.value(0)` | Call method on Pin |
| `digitalRead(2)` | `btn.value()` | Returns 0 or 1 |
{:class="table table-single"}

**Analog I/O:**

| Arduino C++ | MicroPython | Notes |
|-------------|-------------|-------|
| `analogRead(A0)` | `adc = ADC(Pin(26))`<br>`adc.read_u16()` | 16-bit (0-65535) |
| `analogWrite(9, 128)` | `pwm = PWM(Pin(9))`<br>`pwm.duty_u16(32768)` | 16-bit (0-65535) |
| Value: 0-1023 (10-bit) | Value: 0-65535 (16-bit) | Higher resolution |
| PWM: 0-255 (8-bit) | PWM: 0-65535 (16-bit) | Higher resolution |
{:class="table table-single"}

**Timing:**

| Arduino C++ | MicroPython | Notes |
|-------------|-------------|-------|
| `delay(1000)` | `time.sleep(1)` | **SECONDS not milliseconds!** |
| `delay(100)` | `time.sleep(0.1)` | Convert ms to seconds |
| `delayMicroseconds(10)` | `time.sleep_us(10)` | Microseconds |
| `millis()` | `time.ticks_ms()` | Milliseconds since boot |
| `micros()` | `time.ticks_us()` | Microseconds since boot |
| `currentMillis - previousMillis` | `time.ticks_diff(current, previous)` | Handles wrap-around |
{:class="table table-single"}

**Serial/Print:**

| Arduino C++ | MicroPython | Notes |
|-------------|-------------|-------|
| `Serial.begin(9600)` | Not required | Just use print() |
| `Serial.println("Hello")` | `print("Hello")` | Simpler! |
| `Serial.print(value)` | `print(value, end="")` | No newline |
| `Serial.print("Value: " + String(x))` | `print(f"Value: {x}")` | F-strings |
{:class="table table-single"}

**Common Conversions:**

```python
# Arduino delay to MicroPython
delay(1000)           → time.sleep(1)          # 1 second
delay(500)            → time.sleep(0.5)        # 0.5 seconds
delay(100)            → time.sleep(0.1)        # 0.1 seconds
delayMicroseconds(10) → time.sleep_us(10)      # 10 microseconds

# Arduino PWM to MicroPython
analogWrite(pin, 0)    → pwm.duty_u16(0)      # Off
analogWrite(pin, 64)   → pwm.duty_u16(16384)  # 25%
analogWrite(pin, 128)  → pwm.duty_u16(32768)  # 50%
analogWrite(pin, 191)  → pwm.duty_u16(49152)  # 75%
analogWrite(pin, 255)  → pwm.duty_u16(65535)  # 100%

# Arduino ADC to voltage (5V, 10-bit)
voltage = (analogRead(A0) / 1023.0) * 5.0

# MicroPython ADC to voltage (3.3V, 16-bit)
voltage = (adc.read_u16() / 65535) * 3.3
```

## Common Pitfalls and Solutions

**1. Milliseconds vs Seconds**

```python
# WRONG - waits 1000 seconds!
time.sleep(1000)

# RIGHT - waits 1 second
time.sleep(1)

# If you're thinking in milliseconds:
time.sleep_ms(1000)  # 1000 milliseconds = 1 second
```

**2. Boolean Values**

```python
# WRONG - lowercase won't work
flag = true

# RIGHT - capitalize
flag = True
```

**3. Indentation Errors**

```python
# WRONG - inconsistent indentation
if x > 10:
  print("Big")
    print("Really big")  # Error!

# RIGHT - consistent 4 spaces
if x > 10:
    print("Big")
    print("Really big")
```

**4. Pin Numbers**

```python
# Different boards use different pin numbers!

# Pico / Pico 2W - use GP numbers
led = Pin(25, Pin.OUT)  # GP25

# Pico W - built-in LED is special
led = Pin("LED", Pin.OUT)

# Arduino Nano ESP32 - use Arduino numbers
led = Pin(13, Pin.OUT)  # D13
```

**5. ADC Resolution**

```python
# Arduino gives 0-1023
# MicroPython gives 0-65535
# Don't compare directly!

# Convert MicroPython to Arduino range:
arduino_value = adc.read_u16() >> 6  # Shift right 6 bits
```

## When to Use Each Language

**Use MicroPython when:**
- Rapid prototyping and development
- IoT projects with WiFi/networking
- Interactive debugging with REPL
- Complex data processing
- You're more comfortable with Python
- Learning programming/robotics

**Use Arduino C++ when:**
- Maximum performance required
- Ultra-low power battery operation
- Microsecond-level timing critical
- Using Arduino-specific libraries
- Working with existing C++ codebase
- Very tight memory constraints

**Both work great for:**
- LED control and animations
- Sensor reading
- Motor control
- Simple robotics
- Home automation
- Educational projects

## Hardware Recommendations

**Best MicroPython Boards:**

1. **Raspberry Pi Pico W** ($6)
   - Great value, WiFi built-in
   - Huge community support
   - Perfect for learning

2. **Raspberry Pi Pico 2W** ($7)
   - Faster processor than Pico W
   - More memory
   - Future-proof

3. **Arduino Nano ESP32** ($20)
   - Familiar Arduino form factor
   - Powerful ESP32-S3 processor
   - WiFi and Bluetooth

4. **ESP32-C3/C6/S3 boards** ($5-15)
   - Various form factors
   - Great WiFi performance
   - Good for IoT projects

## Recommended Next Courses

**Continue your MicroPython journey:**

{% include gallery.html links="https://www.kevsrobots.com/learn/learning_pathways/micropython.html" images="/assets/img/learn/learn_micropython.png" titles="Micropython Learning Pathway" cols="3" use-links=true noborder=true %}

---

## Additional Resources

**Official Documentation:**
- [MicroPython Documentation](https://docs.micropython.org/)
- [Raspberry Pi Pico Python SDK](https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-python-sdk.pdf)
- [Arduino MicroPython](https://docs.arduino.cc/micropython/)

**Community:**
- [MicroPython Forum](https://forum.micropython.org/)
- [Raspberry Pi Forums](https://forums.raspberrypi.com/)
- [Reddit r/micropython](https://reddit.com/r/micropython)

**Tools:**
- [Thonny IDE](https://thonny.org/) - Best beginner IDE for MicroPython
- [PyCharm with MicroPython plugin](https://www.jetbrains.com/pycharm/)
- [VS Code with Pico-W-Go](https://marketplace.visualstudio.com/items?itemName=paulober.pico-w-go)

**Books:**
- "Programming the Raspberry Pi Pico in MicroPython" by Simon Monk
- "MicroPython for the Internet of Things" by Charles Bell
- "Get Started with MicroPython on Raspberry Pi Pico" by Gareth Halfacree

## Quick Start Template

Save this as your starting point for new MicroPython projects:

```python
"""
Project Name: [Your Project Name]
Description: [Brief description]
Hardware: [Board type and components]
"""

from machine import Pin, PWM, ADC
import time

# Configuration
LED_PIN = 25
BUTTON_PIN = 14

# Hardware setup
led = Pin(LED_PIN, Pin.OUT)
button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_UP)

# Global variables
running = True

def setup():
    """Initialize hardware and print status"""
    print("=" * 50)
    print("Project: [Your Project Name]")
    print("=" * 50)

    # Startup blink
    for i in range(3):
        led.toggle()
        time.sleep(0.2)

    led.off()
    print("Ready!")

def loop():
    """Main program loop"""
    while running:
        # Your code here
        if button.value() == 0:
            led.on()
        else:
            led.off()

        time.sleep(0.1)

def main():
    """Main program entry point"""
    try:
        setup()
        loop()
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        led.off()
        print("Cleanup complete")

if __name__ == '__main__':
    main()
```

## Final Thoughts

**You're now equipped to:**
- Convert any Arduino project to MicroPython
- Choose the right language for your project
- Debug MicroPython code effectively
- Build IoT and robotics projects
- Continue learning advanced topics

**Remember:**
- MicroPython is about rapid development and readability
- Don't be afraid to use the REPL for experimenting
- Start simple, add complexity gradually
- Share your projects with the community!

**The biggest advantage of MicroPython?**

You can type commands directly to your hardware and see instant results. No compile, no upload - just immediate feedback. This makes learning and debugging dramatically faster than traditional Arduino development.

## Your Next Project Ideas

Try building one of these:

1. **Smart Plant Monitor** - Soil moisture sensor, automatic watering, web dashboard
2. **Weather Station** - Temperature, humidity, pressure sensors with data logging
3. **Line-Following Robot** - IR sensors, motor control, autonomous navigation
4. **Home Security System** - Motion sensors, camera, notifications
5. **WiFi-Controlled LED Strip** - RGB LED control via web interface
6. **Temperature-Controlled Fan** - Automatic fan speed based on temperature
7. **Door Sensor with Notifications** - Send alerts when door opens
8. **Solar Tracker** - Light sensors control servo to follow the sun
9. **Game Controller** - Buttons and joysticks over WiFi
10. **Pet Feeder** - Scheduled feeding with servo and timer

## Course Summary

**Lesson 1:** Why choose MicroPython vs Arduino C++
**Lesson 2:** Syntax basics - variables, types, comments
**Lesson 3:** Control flow - if/elif/else, loops, no braces
**Lesson 4:** Functions - def keyword, no setup/loop required
**Lesson 5:** Pin control - Pin class, digital I/O
**Lesson 6:** Analog and PWM - ADC and PWM classes
**Lesson 7:** Timing - seconds vs milliseconds (critical!)
**Lesson 8:** Serial and REPL - print() and interactive debugging
**Lesson 9:** IoT project - WiFi temperature monitor
**Lesson 10:** Robot project - Motor control and sensors
**Lesson 11:** This lesson - reference guide and next steps

## Thank You!

Thank you for completing this course! You've taken a big step in expanding your microcontroller programming skills. MicroPython opens up new possibilities for rapid prototyping, IoT development, and creative projects.

**Keep learning, keep building, and most importantly - have fun!**

If you found this course helpful, consider exploring other courses on this site and joining the maker community. Share your projects, ask questions, and help others learn.

**Happy Making!**

