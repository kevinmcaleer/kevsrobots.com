---
title: Control Flow - If, Loops, and No Braces!
description: Learn how if/else statements, for loops, and while loops work in MicroPython without curly braces - indentation is everything
layout: lesson
type: page
cover: /learn/arduino_to_python/assets/cover.jpg
---

## Control Flow: The Python Way

Control flow - if statements, loops, and logic - works similarly in Python and C++, but with one huge difference: **no curly braces**. Instead, Python uses indentation and colons.

This takes some getting used to, but most programmers find it cleaner once they adjust. Let's explore each control structure side-by-side.

## If Statements

The basic if statement structure changes from braces to colons and indentation:

**Arduino C++:**
```cpp
int temperature = 25;

if (temperature > 30) {
    digitalWrite(LED_PIN, HIGH);
    Serial.println("Hot!");
}
```

**MicroPython:**
```python
temperature = 25

if temperature > 30:
    led.on()
    print("Hot!")
```

Key differences:
- **Colon `:` starts the block** (instead of opening brace)
- **Indentation defines the block** (instead of closing brace)
- No parentheses required (but allowed)

## If-Else Statements

**Arduino C++:**
```cpp
if (temperature > 30) {
    digitalWrite(LED_PIN, HIGH);
} else {
    digitalWrite(LED_PIN, LOW);
}
```

**MicroPython:**
```python
if temperature > 30:
    led.on()
else:
    led.off()
```

Notice the `else:` also needs a colon, and the code below it is indented.

## If-Else If-Else (Elif)

C++ uses `else if`, Python uses `elif`:

**Arduino C++:**
```cpp
if (temperature > 30) {
    Serial.println("Hot");
} else if (temperature > 20) {
    Serial.println("Warm");
} else if (temperature > 10) {
    Serial.println("Cool");
} else {
    Serial.println("Cold");
}
```

**MicroPython:**
```python
if temperature > 30:
    print("Hot")
elif temperature > 20:
    print("Warm")
elif temperature > 10:
    print("Cool")
else:
    print("Cold")
```

`elif` is shorter and cleaner than `else if`. Remember the colon after each condition!

## Comparison Operators

These are identical in both languages:

| Operator | Meaning | Example |
|----------|---------|---------|
| `==` | Equal to | `if x == 10:` |
| `!=` | Not equal | `if x != 10:` |
| `>` | Greater than | `if x > 10:` |
| `<` | Less than | `if x < 10:` |
| `>=` | Greater or equal | `if x >= 10:` |
| `<=` | Less or equal | `if x <= 10:` |

## Logical Operators

Here's where Python differs - it uses words instead of symbols:

**Arduino C++:**
```cpp
if (temperature > 20 && humidity < 80) {
    Serial.println("Comfortable");
}

if (temperature > 35 || humidity > 90) {
    Serial.println("Uncomfortable");
}

if (!doorOpen) {
    Serial.println("Door is closed");
}
```

**MicroPython:**
```python
if temperature > 20 and humidity < 80:
    print("Comfortable")

if temperature > 35 or humidity > 90:
    print("Uncomfortable")

if not door_open:
    print("Door is closed")
```

| C++ | Python | Meaning |
|-----|--------|---------|
| `&&` | `and` | Both conditions true |
| `||` | `or` | Either condition true |
| `!` | `not` | Negation |

Python's word-based operators are more readable once you get used to them.

## While Loops

While loops are nearly identical, just swap braces for colons:

**Arduino C++:**
```cpp
int count = 0;

while (count < 10) {
    Serial.println(count);
    count++;
}
```

**MicroPython:**
```python
count = 0

while count < 10:
    print(count)
    count += 1  # Python has +=, but no ++ operator
```

**Important:** Python doesn't have `++` or `--` operators. Use `+= 1` or `-= 1` instead.

## Infinite Loops

In Arduino, your main code runs in `loop()` which repeats forever. In MicroPython, you create an infinite loop explicitly:

**Arduino C++:**
```cpp
void loop() {
    // This repeats forever automatically
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    delay(1000);
}
```

**MicroPython:**
```python
while True:  # Explicit infinite loop
    led.on()
    time.sleep(1)
    led.off()
    time.sleep(1)
```

`while True:` is the Python equivalent of Arduino's `loop()`. You'll use this pattern in almost every MicroPython program.

> **CircuitPython File Naming: boot.py and code.py**
>
> If you're using CircuitPython instead of MicroPython, there's an important difference in how you structure your main program:
>
> **MicroPython:**
> - **boot.py** - Runs first on startup (optional, for configuration)
> - **main.py** - Your main program code (runs automatically after boot.py)
>
> **CircuitPython:**
> - **boot.py** - Runs first on startup (same as MicroPython)
> - **code.py** - Your main program code (instead of main.py)
>
> **The key difference:** CircuitPython uses `code.py` as the default filename instead of `main.py`.
>
> **Auto-reload magic:**
> - When you save `code.py` on CircuitPython, your board automatically restarts and runs the new code
> - This happens because CircuitPython shows up as a USB drive - when you save the file, it detects the change
> - MicroPython requires you to manually restart or use Thonny's "Run" button
>
> **Example - Same blink code, different workflow:**
>
> MicroPython (`main.py`):
> ```python
> from machine import Pin
> import time
>
> led = Pin(25, Pin.OUT)
>
> while True:
>     led.on()
>     time.sleep(1)
>     led.off()
>     time.sleep(1)
> ```
>
> CircuitPython (`code.py`):
> ```python
> import board
> import digitalio
> import time
>
> led = digitalio.DigitalInOut(board.LED)
> led.direction = digitalio.Direction.OUTPUT
>
> while True:
>     led.value = True
>     time.sleep(1)
>     led.value = False
>     time.sleep(1)
> ```
>
> **Practical tip:** If you're switching between platforms:
> - You can have both `main.py` and `code.py` on your board
> - MicroPython will run `main.py`, CircuitPython will run `code.py`
> - Just remember which platform you're using!

## For Loops

For loops work very differently in Python. There's no C-style `for (int i=0; i<10; i++)` syntax.

**Arduino C++:**
```cpp
// Count from 0 to 9
for (int i = 0; i < 10; i++) {
    Serial.println(i);
}

// Blink LED 5 times
for (int i = 0; i < 5; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
}
```

**MicroPython:**
```python
# Count from 0 to 9
for i in range(10):
    print(i)

# Blink LED 5 times
for i in range(5):
    led.on()
    time.sleep(0.5)
    led.off()
    time.sleep(0.5)
```

Python's `range()` function generates numbers:
- `range(10)` → 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
- `range(5, 10)` → 5, 6, 7, 8, 9
- `range(0, 10, 2)` → 0, 2, 4, 6, 8 (step by 2)

## Iterating Over Lists

Python's for loops really shine when working with lists:

**Arduino C++:**
```cpp
int pins[] = {13, 12, 11, 10};
int numPins = 4;

for (int i = 0; i < numPins; i++) {
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], HIGH);
}
```

**MicroPython:**
```python
pins = [13, 12, 11, 10]

for pin_num in pins:
    pin = Pin(pin_num, Pin.OUT)
    pin.on()
```

No index needed! Python iterates directly over the list items. Much cleaner.

## Break and Continue

These work identically in both languages:

**Arduino C++:**
```cpp
// Break - exit loop early
for (int i = 0; i < 100; i++) {
    if (digitalRead(BUTTON_PIN)) {
        break;  // Exit loop when button pressed
    }
}

// Continue - skip to next iteration
for (int i = 0; i < 10; i++) {
    if (i % 2 == 0) {
        continue;  // Skip even numbers
    }
    Serial.println(i);
}
```

**MicroPython:**
```python
# Break - exit loop early
for i in range(100):
    if button.value():
        break  # Exit loop when button pressed

# Continue - skip to next iteration
for i in range(10):
    if i % 2 == 0:
        continue  # Skip even numbers
    print(i)
```

## Switch Statements: Python Doesn't Have Them!

Arduino has switch/case. Python doesn't. Use if/elif instead:

**Arduino C++:**
```cpp
int mode = 2;

switch (mode) {
    case 1:
        Serial.println("Mode 1");
        break;
    case 2:
        Serial.println("Mode 2");
        break;
    case 3:
        Serial.println("Mode 3");
        break;
    default:
        Serial.println("Unknown");
}
```

**MicroPython:**
```python
mode = 2

if mode == 1:
    print("Mode 1")
elif mode == 2:
    print("Mode 2")
elif mode == 3:
    print("Mode 3")
else:
    print("Unknown")
```

**Alternative: Dictionary dispatch** (advanced pattern):

```python
mode = 2

actions = {
    1: lambda: print("Mode 1"),
    2: lambda: print("Mode 2"),
    3: lambda: print("Mode 3")
}

actions.get(mode, lambda: print("Unknown"))()
```

This is more Pythonic but less beginner-friendly. Stick with if/elif for now.

## Nested Control Structures

Indentation makes nesting very clear:

**Arduino C++:**
```cpp
if (temperature > 25) {
    if (humidity > 60) {
        digitalWrite(FAN_PIN, HIGH);
        for (int i = 0; i < 3; i++) {
            digitalWrite(LED_PIN, HIGH);
            delay(100);
            digitalWrite(LED_PIN, LOW);
            delay(100);
        }
    }
}
```

**MicroPython:**
```python
if temperature > 25:
    if humidity > 60:
        fan.on()
        for i in range(3):
            led.on()
            time.sleep(0.1)
            led.off()
            time.sleep(0.1)
```

Each nested level adds 4 spaces of indentation. The structure is visually clear.

## Complete Example: Traffic Light

Let's put it all together with a traffic light controller:

**Arduino C++:**
```cpp
int redPin = 11;
int yellowPin = 12;
int greenPin = 13;

void setup() {
    pinMode(redPin, OUTPUT);
    pinMode(yellowPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
}

void loop() {
    // Red light
    digitalWrite(redPin, HIGH);
    delay(5000);
    digitalWrite(redPin, LOW);

    // Green light
    digitalWrite(greenPin, HIGH);
    delay(5000);
    digitalWrite(greenPin, LOW);

    // Yellow light
    digitalWrite(yellowPin, HIGH);
    delay(2000);
    digitalWrite(yellowPin, LOW);
}
```

**MicroPython:**
```python
from machine import Pin
import time

red = Pin(11, Pin.OUT)
yellow = Pin(12, Pin.OUT)
green = Pin(13, Pin.OUT)

while True:
    # Red light
    red.on()
    time.sleep(5)
    red.off()

    # Green light
    green.on()
    time.sleep(5)
    green.off()

    # Yellow light
    yellow.on()
    time.sleep(2)
    yellow.off()
```

## Try It Yourself

**Exercise 1: Temperature Warning System**

Write MicroPython code that:
- Reads a temperature variable
- If temp > 30: print "TOO HOT" and turn on red LED
- If temp 20-30: print "COMFORTABLE" and turn on green LED
- If temp < 20: print "TOO COLD" and turn on blue LED

**Answer:**

```python
from machine import Pin

temperature = 25  # Simulate sensor reading

red_led = Pin(10, Pin.OUT)
green_led = Pin(11, Pin.OUT)
blue_led = Pin(12, Pin.OUT)

if temperature > 30:
    print("TOO HOT")
    red_led.on()
    green_led.off()
    blue_led.off()
elif temperature >= 20:
    print("COMFORTABLE")
    red_led.off()
    green_led.on()
    blue_led.off()
else:
    print("TOO COLD")
    red_led.off()
    green_led.off()
    blue_led.on()
```

**Exercise 2: Button Counter**

Create a program that:
- Counts button presses
- Prints the count each time
- If count reaches 10, print "MAX REACHED" and reset to 0
- Blinks an LED for each press

**Answer:**

```python
from machine import Pin
import time

button = Pin(14, Pin.IN, Pin.PULL_UP)
led = Pin(25, Pin.OUT)

count = 0
last_state = 1

while True:
    current_state = button.value()

    # Detect button press (high to low transition)
    if last_state == 1 and current_state == 0:
        count += 1
        print(f"Count: {count}")

        # Blink LED
        led.on()
        time.sleep(0.1)
        led.off()

        # Check for max
        if count >= 10:
            print("MAX REACHED")
            count = 0

    last_state = current_state
    time.sleep(0.05)  # Debounce delay
```

**Exercise 3: Loop Practice**

Write a program that:
- Blinks an LED 10 times
- Each blink should be faster than the last (start at 1 second, decrease by 0.1s)
- Print the delay time before each blink

**Answer:**

```python
from machine import Pin
import time

led = Pin(25, Pin.OUT)

for i in range(10):
    delay = 1.0 - (i * 0.1)
    print(f"Blink {i+1}, delay: {delay}s")

    led.on()
    time.sleep(delay)
    led.off()
    time.sleep(delay)
```

---
