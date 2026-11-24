---
layout: lesson
title: Operators
author: Kevin McAleer
type: page
cover: /learn/micropython/assets/micropython.jpg
date: 2022-12-04
previous: 13_loops.html
next: 15_functions.html
description: Arithmetic, comparison, assignment and logical operators
percent: 80
duration: 6
quiz_total: 57
date_updated: 2026-11-24
navigation:
- name: Learn MicroPython - The basics
- content:
  - section: Overview
    content:
    - name: Introduction
      link: 00_intro.html
    - name: Learn MicroPython Introduction Video
      link: 00_videos.html
  - section: Introduction
    content:
    - name: Why is it called MicroPython?
      link: 01_why_is_it_called_micropython.html
    - name: Where to get MicroPython
      link: 02_where_to_get_micropython.html
    - name: How to Install MicroPython
      link: 03_installing_micropython.html
    - name: Why use MicroPython?
      link: 04_why_use_micropython.html
    - name: Python Development Environments
      link: 05_ides.html
    - name: Our first program
      link: 06_our_first_program.html
    - name: Example 01
      link: 07_hello_world.html
  - section: Variables and Reserved Words
    content:
    - name: Variables and Constants
      link: 08_variables.html
    - name: Example 02
      link: 09_example02.html
    - name: Values & Variables Types
      link: 10_values_and_variable_types.html
    - name: Reserved Words
      link: 11_reserved_words.html
  - section: Controlling the Flow
    content:
    - name: If, elif, else
      link: 12_if_elif_else.html
    - name: Loops
      link: 13_loops.html
    - name: Operators
      link: 14_operators.html
  - section: Functions and Modules
    content:
    - name: Functions
      link: 15_functions.html
    - name: The REPL
      link: 16_repl.html
    - name: Modules
      link: 17_modules.html
  - section: Summary and Review
    content:
    - name: Summary and Review
      link: 18_summary.html
---


![Cover photo of a telephone operator from the 1920s](assets/operators.jpg){:class="cover"}

---

By the end of this lesson you will be able to use arithmetic, comparison, assignment and logical operators to make a robot calculate and decide.

Operators are symbols such as `+` and `-` that do something with values. There are several families of them. Don't worry about memorising them, you'll pick them up as you use them, and you probably know a few already.

## Arithmetic operators

In these examples `a = 10` and `b = 3`.

| Operator | Symbol | Example | Result |
|---|:--:|---|---|
| Addition | `+` | `a + b` | `13` |
| Subtraction | `-` | `a - b` | `7` |
| Multiplication | `*` | `a * b` | `30` |
| Division | `/` | `a / b` | `3.3333...` (always a `float`) |
| Floor division | `//` | `a // b` | `3` (the whole number part) |
| Modulus | `%` | `a % b` | `1` (the remainder) |
| Exponent | `**` | `a ** b` | `1000` (10 to the power 3) |
{:class="table table-single"}

Here they are working for a robot that drives wheels at different speeds:

{% capture snakie_code %}
left_speed = 50
right_speed = 30

total = left_speed + right_speed
average = total / 2
difference = left_speed - right_speed
print('total', total, 'average', average, 'difference', difference)

# which of 4 sensors comes next, wrapping back to 0?
sensor = 3
next_sensor = (sensor + 1) % 4
print('next sensor is', next_sensor)
{% endcapture %}
{% include snakie.html code=snakie_code name="operators_1.py" %}

The `%` modulus operator is handy for "wrap around" logic, like cycling through four sensors, or doing something every 10th time.

## Comparison operators

Comparison operators compare two values and give `True` or `False`. We met some in the `if` lesson.

| Operator | Symbol | Example with `a = 10`, `b = 20` | Result |
|---|:--:|---|---|
| Equal | `==` | `a == b` | `False` |
| Not equal | `!=` | `a != b` | `True` |
| Greater than | `>` | `a > b` | `False` |
| Less than | `<` | `a < b` | `True` |
| Greater or equal | `>=` | `a >= b` | `False` |
| Less or equal | `<=` | `a <= b` | `True` |
{:class="table table-single"}

## Assignment operators

`=` assigns a value. The others are shortcuts that change a variable using its own value:

| Operator | Example | Same as |
|:--:|---|---|
| `=` | `a = 10` | puts `10` in `a` |
| `+=` | `a += 5` | `a = a + 5` |
| `-=` | `a -= 5` | `a = a - 5` |
| `*=` | `a *= 2` | `a = a * 2` |
| `/=` | `a /= 2` | `a = a / 2` |
| `%=` | `a %= 3` | `a = a % 3` |
| `//=` | `a //= 3` | `a = a // 3` |
| `**=` | `a **= 2` | `a = a ** 2` |
{:class="table table-single"}

```python
speed = 40
speed += 10     # speed is now 50
speed *= 2      # speed is now 100
print(speed)
```

## Logical operators

Logical operators join conditions together, using the Boolean logic from lesson 10:

| Operator | Meaning | Example |
|:--:|---|---|
| `and` | both must be `True` | `battery > 20 and path_clear` |
| `or` | at least one is `True` | `left_bump or right_bump` |
| `not` | flips `True` and `False` | `not moving` |
{:class="table table-single"}

{% capture snakie_code %}
battery = 80
path_clear = True
left_bump = False
right_bump = True

if battery > 20 and path_clear:
    print('Drive forward')

if left_bump or right_bump:
    print('Back up, we hit something')

if not path_clear:
    print('Turn around')
{% endcapture %}
{% include snakie.html code=snakie_code name="operators_2.py" %}

## Membership and identity

`in` checks whether a value is inside a list, text or dictionary. `is` checks whether two names point at the very same thing, and is most often used with `None`:

```python
sensors = ["left", "centre", "right"]
print("left" in sensors)
print("rear" not in sensors)

reading = None
print(reading is None)
```

## Bitwise operators

Bitwise operators work on the individual 1s and 0s of a number: `&` (and), `|` (or), `^` (exclusive or), `~` (flip), `<<` and `>>` (shift). You'll use them later to work with registers and sensor data. Here is a peek, with `a = 60` (`0011 1100`) and `b = 13` (`0000 1101`):

```python
a = 60
b = 13
print(a & b, a | b, a ^ b, a << 2, a >> 2)
```

This prints `12 61 49 240 15`.

## Try it Yourself

1. Work out `17 // 5` and `17 % 5` in your head, then check in your editor.
2. Write a program that stores a wheel diameter of 6.5 and calculates the circumference by multiplying by `3.14159`.
3. Use `and` and `or` to print `Go` only when the battery is above 20 and neither bumper is pressed.
4. Challenge: use `%` to print `tick` every time a loop counter is a multiple of 5, as the counter counts to 20.

## Common Issues

- **Problem**: My answer is `3.0` when I expected `3`.
- **Solution**: Use `//` for a whole number answer, or wrap the result in `int()`.
- **Why**: A single `/` always gives a decimal `float`.

- **Problem**: `ZeroDivisionError: divide by zero`.
- **Solution**: Check the bottom number is not `0` before you divide.
- **Why**: Dividing by zero has no answer.

- **Problem**: `if a = 5:` gives a syntax error.
- **Solution**: Use `==` to compare.
- **Why**: `=` assigns a value and can't be used as a test.

## Check your understanding

{% quiz %}
? What is `17 % 5`?
- [ ] `3`
- [x] `2`
- [ ] `3.4`
> `%` gives the remainder: 17 divided by 5 is 3 remainder 2.

? What is `speed += 10` the same as?
- [ ] `speed = 10`
- [x] `speed = speed + 10`
- [ ] `speed == 10`
> `+=` adds to the variable's current value.

? When is `left_bump or right_bump` `True`?
- [ ] Only when both are `True`
- [x] When at least one is `True`
- [ ] Only when both are `False`
> `or` needs just one side to be `True`.
{% endquiz %}
