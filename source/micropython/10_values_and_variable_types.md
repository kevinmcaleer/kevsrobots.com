---
title: Values & Variables Types
description: Numbers, text, lists and dictionaries
layout: lesson
type: page
date_updated: 2026-11-24
---

![Cover photo of a yellow typewriter](assets/types.jpg){:class="cover"}

---

By the end of this lesson you will be able to name the main types of value in MicroPython, check a value's type with `type()`, and convert between types.

## Data types

Programs work with numbers and text, adding and subtracting, asking questions and processing answers. MicroPython labels each value with a **type** so it knows what you are allowed to do with it. You can add two numbers, but you can't add a number to a name.

Here are the types you'll use most, with robot examples:

| Type | What it holds | Example |
|:-----|:--------------|:--------|
| `int` | A whole number, positive or negative | `60`, `-5` |
| `float` | A number with a decimal point | `3.14`, `-2.5` |
| `str` | Text, wrapped in quotes | `'Bolt'`, `"hello"` |
| `bool` | `True` or `False` | `True` |
| `list` | An ordered collection you can change | `["left", "centre", "right"]` |
| `tuple` | An ordered collection you can't change | `(10, 20)` |
| `dict` | A dictionary of key and value pairs | `{"name": "Bolt", "speed": 60}` |
{:class="table table-single"}

There are more types, such as sets and bytes, which we'll meet in later courses.

When we assign a value to a variable, the variable takes the value's type. Let's see them all with `type()`:

```python
speed = 60
distance = 12.5
robot_name = "Bolt"
moving = True
sensors = ["left", "centre", "right"]
position = (10, 20)
settings = {"name": "Bolt", "speed": 60}

for value in (speed, distance, robot_name, moving, sensors, position, settings):
    print(value, type(value))
```

(That `for` loop repeats the print for each value. Loops are in lesson 13.) The shell prints something like `60 <class 'int'>`.

## Getting at values

A list uses a position number, which starts at `0`. A dictionary uses a key:

```python
sensors = ["left", "centre", "right"]
settings = {"name": "Bolt", "speed": 60}

print(sensors[0])
print(settings["name"])

sensors.append("rear")
settings["speed"] = 80
print(sensors, settings)
```

## Changing type

Sometimes we need a different type, for example when `input()` gives us text. This is called `casting`:

```python
text = "42"
number = int(text)
half = number / 2
print(number + 1, half, str(number) + " cm")
```

Notice `/` always gives a `float` (`21.0`). Use `int()` to change type, `float()` for decimals and `str()` for text.

> ## Implied Types
>
> In languages such as `C/C++` you must tell the compiler the type of a variable before you use it:
>
> ```c++
> int a;
> float b;
> a = 1;
> b = 2.3;
> ```
>
> In `MicroPython` you don't. It works the type out from the value you assign, which is called `dynamic typing`:
>
> ```python
> a = 1
> b = 2.3
> ```
{:.bg-blue}

The flip side is that a variable can later be given a value of a different type, which can lead to surprises if you forget.

## George Boole and the bool

English mathematician [George Boole](https://en.wikipedia.org/wiki/George_Boole) wrote `The Laws of Thought` (1854), showing that logic can be expressed in symbols with an output of `True` or `False`. It is now called `Boolean logic`, and all modern computing is based on it. This is the truth table for `and`, where both inputs must be `True` for the output to be `True`:

| input a | input b | output |
|:--:|:--:|:--:|
| `False` | `False` | `False` |
| `True` | `False` | `False` |
| `False` | `True` | `False` |
| `True` | `True` | `True` |
{:class="table table-single"}

We'll use this in the [Operators lesson](14_operators.html).

## Try it Yourself

1. Create one variable of each type in the table, and print its `type()`.
2. Make a list of three sensor names, then add a fourth with `append()` and print the list.
3. Make a dictionary for a robot with a name and a speed. Change the speed and print it.
4. Challenge: ask for a number with `input()`, convert it, and print half of it.

## Common Issues

- **Problem**: `TypeError: unsupported types for __add__: 'int', 'str'`.
- **Solution**: Convert one side so both match, such as `str(number) + " cm"`.
- **Why**: You can't add a number to text, so one has to change type first.

- **Problem**: `IndexError: list index out of range`.
- **Solution**: Remember positions start at 0, so a list of three has positions 0, 1 and 2.
- **Why**: You asked for a position that doesn't exist.

- **Problem**: `TypeError: 'tuple' object doesn't support item assignment`.
- **Solution**: Use a list if you need to change items.
- **Why**: Tuples can't be changed after they are created.

## Check your understanding

{% quiz %}
? What type is the value `2.5`?
- [ ] `int`
- [x] `float`
- [ ] `str`
> A number with a decimal point is a `float`.

? How do you get the first item of a list called `sensors`?
- [ ] `sensors[1]`
- [x] `sensors[0]`
- [ ] `sensors.first`
> List positions start at 0.

? What will `type(True)` show?
- [ ] `<class 'int'>`
- [x] `<class 'bool'>`
- [ ] `<class 'str'>`
> `True` and `False` are booleans.
{% endquiz %}
