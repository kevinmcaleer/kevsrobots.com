---
title: Syntax Basics - No More Semicolons!
description: Learn the fundamental syntax differences between Arduino C++ and MicroPython including variables, types, comments, f-strings and indentation
layout: lesson
type: page
cover: /learn/arduino_to_python/assets/banners/02_syntax_basics.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to write simple Python statements, variables and formatted print lines, and spot the C++ habits that break them.

## Three big changes

Moving from C++ to MicroPython means unlearning a few habits:

1. No semicolons at the end of lines
2. Indentation defines code blocks, not curly braces
3. You do not declare variable types

## Semicolons and variables

In C++ every statement ends with `;` and every variable has a type. In Python a line break ends the statement and the type comes from the value.

**Arduino C++:**
```cpp
int counter = 0;
float temperature = 23.5;
bool isRunning = true;
String message = "Hello";
```

**MicroPython:**

{% capture snakie_code %}
counter = 0              # int
temperature = 23.5       # float
is_running = True        # bool: capital T!
message = "Hello"        # str
print(counter, temperature, is_running, message)
{% endcapture %}
{% include snakie.html code=snakie_code name="syntax_basics_1.py" %}

Things to remember:

- `True` and `False` start with a capital letter
- Single and double quotes both make strings, and there is no separate `char` type
- Python style is `snake_case` for variables and functions, not `camelCase`

A Python variable can change type: `value = 42` and then `value = "Hello"` is allowed. That flexibility is handy, and it is also a way to hide bugs, so keep each variable to one job.

## Comments and constants

**Arduino C++:**
```cpp
// single line
/* multi
   line */
const int LED_PIN = 13;
#define MAX_TEMP 30
```

**MicroPython:**

{% capture snakie_code %}
# Single-line comment
LED_PIN = 13     # ALL_CAPS by convention means "constant"
MAX_TEMP = 30
print(LED_PIN, MAX_TEMP)
{% endcapture %}
{% include snakie.html code=snakie_code name="syntax_basics_2.py" %}

Python has no `/* */` comment. Most people just put `#` at the start of each line. Python also does not enforce constants: `MAX_TEMP = 99` later would work, so the capital letters are a promise to yourself.

## Indentation is the syntax

In C++, braces define a block and indentation is only for readability. In Python the indentation *is* the block, and a colon `:` opens it.

**Arduino C++:**
```cpp
if (temperature > 25) {
    digitalWrite(13, HIGH);
    Serial.println("Hot!");
}
```

**MicroPython:**

{% capture snakie_code %}
temperature = 30

if temperature > 25:
    print("Fan on")
    print("Hot!")
print("This line runs either way")
{% endcapture %}
{% include snakie.html code=snakie_code name="syntax_basics_3.py" %}

Rules: use four spaces per level, never mix tabs and spaces, and every line in a block must line up. Thonny indents for you after a colon when you press Enter.

## F-strings replace string gymnastics

Arduino C++ makes you glue strings and numbers together with `String()` or several `Serial.print()` calls. Python's f-strings drop values straight into text.

**Arduino C++:**
```cpp
Serial.print("Temp: ");
Serial.print(temperature, 1);
Serial.println("C");
```

**MicroPython:**

{% capture snakie_code %}
temperature = 23.456
humidity = 67
print(f"Temp: {temperature:.1f}C, Humidity: {humidity}%")
{% endcapture %}
{% include snakie.html code=snakie_code name="syntax_basics_4.py" %}

`:.1f` means one decimal place, just like the second argument to `Serial.print()`. Type conversion is simple too: `int("123")`, `float("3.5")`, `str(42)`.

## Putting it together

Here is a sensor sketch and its MicroPython twin. The Pico's ADC returns 0 to 65535 (we cover why in lesson 6), so the threshold changes.

**Arduino C++:**
```cpp
int sensorPin = A0;
int ledPin = 13;
int threshold = 500;

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  int sensorValue = analogRead(sensorPin);
  Serial.println(sensorValue);
  digitalWrite(ledPin, sensorValue > threshold);
  delay(1000);
}
```

**MicroPython:**
```python
from machine import Pin, ADC
import time

sensor = ADC(Pin(26))        # ADC0 on a Pico
led = Pin("LED", Pin.OUT)
threshold = 32000            # about half of 65535

while True:
    sensor_value = sensor.read_u16()
    print(f"Sensor: {sensor_value}")
    led.value(sensor_value > threshold)
    time.sleep(1)
```

## Try it Yourself

1. Declare a variable `speed = 50` and print `Speed: 50 percent` with an f-string.
2. Convert `float temperature = 21.37;` to Python and print it with one decimal place.
3. The program below has C++ habits in it. Find the C++ habits, then fix it.
4. Challenge: write a program that reads the ADC on GP26 and prints both the raw value and the voltage (raw divided by 65535, times 3.3) to two decimal places.

<!-- code-check: skip -->
```python
from machine import Pin;
import time

led = Pin("LED", Pin.OUT);

while True
    led.on();
    time.sleep(1);
    led.off();
    time.sleep(1);
```
{: .no-snakie }

The mistakes are the semicolons, and the missing colon after `while True`. Remove all the semicolons and add the colon.

## Common Issues

- **Problem**: `SyntaxError: invalid syntax` on a `while` or `if` line.
- **Solution**: Check for a missing colon at the end of the line.
- **Why**: C++ uses braces to open a block, Python uses the colon, and it is easy to forget when your fingers expect `{`.

- **Problem**: `IndentationError: unexpected indent` or `unindent does not match`.
- **Solution**: Make every line in a block use the same number of spaces, and never mix tabs and spaces.
- **Why**: Indentation is how Python knows where a block starts and ends.

- **Problem**: `NameError: name 'true' isn't defined`.
- **Solution**: Write `True` with a capital T (and `False`, `None`).
- **Why**: Python's built-in constants are capitalised.

## Check your understanding

{% quiz %}
? What marks the start of a code block in Python?
- [ ] An opening curly brace
- [x] A colon at the end of the line, followed by indented lines
- [ ] A semicolon
> Python uses a colon and indentation instead of braces.

? Which is the correct way to write a boolean in MicroPython?
- [ ] `true`
- [ ] `TRUE`
- [x] `True`
> The built-in values are `True` and `False`.

? What does `print(f"Temp: {t:.1f}C")` do when `t = 23.456`?
- [ ] Prints `Temp: 23.456C`
- [x] Prints `Temp: 23.5C`
- [ ] Raises an error
> `.1f` formats the number with one decimal place.

? How do you declare `int counter = 0;` in Python?
- [ ] `int counter = 0`
- [x] `counter = 0`
- [ ] `var counter = 0;`
> Python infers the type from the value, so there is no type keyword.
{% endquiz %}
