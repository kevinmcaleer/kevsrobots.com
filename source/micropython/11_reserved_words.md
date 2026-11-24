---
title: Reserved Words
description: All the words that have a special meaning in Python
layout: lesson
type: page
date_updated: 2026-11-24
---

![Reserved sign on a table](assets/reserved.jpg){:class="cover"}

---

By the end of this lesson you will be able to explain what a reserved word is, spot one in a program, and avoid using one as a variable name.

## Special keywords

MicroPython has a list of special keywords that can't be used as names for variables, because each one has a job in the language. They are called `reserved words`. When you write `if`, MicroPython knows you are about to make a decision, so it can't also mean "a box called if".

Here is the full list of reserved words in MicroPython:

| Reserved Words |         |          |            |          |            |
|:--------------:|:-------:|:--------:|:----------:|:--------:|:----------:|
|     `and`      |  `as`   | `assert` |  `async`   | `await`  |  `break`   |
|    `class`     | `continue` | `def` |   `del`    |  `elif`  |   `else`   |
|    `except`    | `finally` | `for`  |   `from`   | `global` |    `if`    |
|    `import`    |  `in`   |   `is`   |  `lambda`  | `nonlocal` |  `not`   |
|      `or`      | `pass`  | `raise`  |  `return`  |  `try`   |  `while`   |
|     `with`     | `yield` |  `True`  |  `False`   |  `None`  |            |
{:class="table table-single"}

You don't need to memorise them. You've already met some (`if`, `else`, `import`, `True`), and we'll meet most of the others in this course and the later ones. Your editor helps too, because it colours reserved words differently from your own names.

## What happens if you use one?

Try to name a variable after a reserved word and MicroPython stops straight away:

<!-- code-check: skip -->
```python
class = "robot"
```

You get a `SyntaxError: invalid syntax`. The fix is simply to choose a more descriptive name, such as `robot_class` or `kind`:

```python
robot_kind = "rover"
print(robot_kind)
```

## Sneaky names that are not reserved

Some words, such as `print`, `input`, `type` and `list`, are not reserved, but they are the names of built-in functions. MicroPython lets you reuse them, which is a trap, because the function stops working:

<!-- code-check: skip -->
```python
print = 5
print("Hello")
```

The second line fails with `TypeError: 'int' object isn't callable`, because `print` is now a box holding 5. If this happens, press the board's reset or restart your session to get `print` back. A good habit is to use names like `speed_list` or `sensor_type` instead.

## Try it Yourself

1. Look at the table above. Find three words that you recognise from earlier lessons.
2. Type `for` as a variable name in your editor. What does it say?
3. Rename a reserved-word variable so the program runs.
4. Challenge: think of three good names for a variable holding a robot's battery voltage, and choose the best.

## Common Issues

- **Problem**: `SyntaxError: invalid syntax` on a line that looks fine.
- **Solution**: Check whether the variable name is a reserved word, such as `class`, `from` or `in`.
- **Why**: Reserved words are part of the language, so they can't be used as names.

- **Problem**: `TypeError: 'int' object isn't callable` when I call a function that worked before.
- **Solution**: You probably gave a variable the same name as a function, such as `print`. Rename it and restart.
- **Why**: Your variable replaced the built-in function with a number.

## Check your understanding

{% quiz %}
? What is a reserved word?
- [ ] A word you must always type in capitals
- [x] A word with a special meaning that you can't use as a variable name
- [ ] A word you can only use once
> Reserved words are part of the language, such as `if` and `while`.

? Which of these is a reserved word?
- [ ] `speed`
- [x] `while`
- [ ] `robot`
> `while` is in the list of reserved words.

? What happens if you run `print = 5` and then `print("Hi")`?
- [ ] It prints Hi
- [x] You get a `TypeError` because `print` is now the number 5
- [ ] The program closes
> Reusing a built-in function's name replaces it.
{% endquiz %}
