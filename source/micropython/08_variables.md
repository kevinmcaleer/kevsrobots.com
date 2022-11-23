--- 
title: Variables and Constants
description: A Place to store values
layout: lesson
---

![Pigeon Holes Photo](assets/pigeon_holes.jpg){:class="cover"}

##### Example

```python
a = 1
```

When we want to store a value we need to use a `variable`. In the example we looked at above we stored the number `1` in the variable `a`, we assigned it by using the equals sign.

When we think about variables, we can imagine they are like little boxes that store things, and we give those boxes names so that it makes referring to them easier. We can put things into the boxes, look what’s in them or compare what’s in them with another box.

We could just give our variables a numbered label but that would make it difficult to remember what is in each one. By naming our variables we can easily understand what we expect the variable to contain. For example if we want to storge the age of a person we could use a variable named `age`.

```python
age = 21
```

---

> ## MicroPython Facts
>
> Like Python, MicroPython variables don’t actually store the value, they just point to the thing that does.
{:class="blockquote bg-blue"}

---

## Constants

If `variables` contain values that change, `Constants` contain values that dont change.
For example, if we wanted to store the value of `pi`, the value of `pi` does not change, so we can store this in a special type of object called a `constant`.

##### Example

```python
PI = const(3.141592654)
```

In the example above we use the `const()` function to define the value `3.141592654` as the constant `PI`. The convention is that constants are always in uppercase.

> ## Const()
>
> `const()` is a specific to MicroPython, in regular python constants are treated as varibles, just in upper case.
