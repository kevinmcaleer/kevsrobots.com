---
title: Values & Variables Types
description: NUMBERS, TEXT, LISTS AND DICTIONARIES
layout: lesson
---

![Cover photo of a yellow typewritter](assets/type.jpg){:class="cover"}

Programs often need to work with numbers and text, adding and subtracting, asking questions and processing answers.

MicroPython is able to label different types of values so that it can make different commands available to us.

The different types of values are listed below:

{:.caption}
List of MicroPython data types

| Text type: | `str`
| Numeric types: | `int`, `float`, `complex`|
| Lists and sequences: | `list`, `tuple` |
| Dictionaries (mapping of key-value pairs) | `dict` |
| Set types: | `set`, `frozenset` |
| True/False (Boolean) type: | `bool`|
| Binary types: | `bytes`, `bytearray`, `memoryview` |
| None type: | `NoneType`|
{:class="table-w100 table table-bordered"}

| Value Types | Description                                                  | Example                                                |
|:------------|:-------------------------------------------------------------|--------------------------------------------------------|
| `int`       | A whole number, which can be positive or negative            | `1, 2, 3, 4, -500, 1000`                               |
| `float`     | A decimal number, which can be positive or negative          | `3.14, -2.5, 1.66666 `                                 |
| `str`       | A line of text                                               | `'hello world', 'my name is Kevin'  `                  |
| `list`      | A collection of words or numbers, or objects                 | `[‘cat’,’dog’,’fish’,’gecko’], `or `[1,2,3,5,8,13,21]` |
| `dict`      | A dictionary - a list, with pairs of values                  | `{‘name’:’Kevin’, ‘address’:’UK’, ‘age’:45} `          |
| 'tuple'     | a collection of                                              |                                                        |
| `object`    | An instance of a class (we’ll cover this in a later session) | Object of `'Cat'` Class                                |
{:class="table-w100 table table-bordered"}

When we assign a value to a variable, it is said to have the a *type*.
e.g.

```python
a = 1
b = 2.1
c = "hello"
```

In this example `a` contains the value `1` and is of type `integer`, `b` contains the value `2.1` and is of type `float` (because it has a decimal point), and `c` contains `hello` and is of type `string` because it contains a text wrapped inside some speech marks.

---