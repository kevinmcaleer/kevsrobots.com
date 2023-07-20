---
title: Working with Files in Python
description: Learn how to read from and write to files in Python, handling both text and binary files.
layout: lesson
cover: assets/5.png
---

![cover image]({{page.cover}}){:class="cover"}

## Introduction

Being able to interact with files is an important skill for any programmer. In this lesson, we'll learn how to read from and write to files in Python, including both text and binary files.

---

## Learning Objectives

- Understand how to open and close files.
- Understand how to read from and write to text files.
- Understand how to read from and write to binary files.

---

### Opening and Closing Files

You can open a file using the built-in `open()` function. This function returns a file object and is most commonly used with two arguments: `open(filename, mode)`. The `mode` argument is a string that contains multiple characters representing how you want to open the file.

```python
# Open a file for writing
f = open("test.txt", "w")

# Always remember to close files
f.close()
```

The most commonly used modes are:

- `'r'`: read (default)
- `'w'`: write (creates a new file or overwrite existing content)
- `'a'`: append (adds new data to the end of the file)
- `'b'`: binary mode
- `'t'`: text mode (default)
- `'+'`: read and write

---

### Reading and Writing Text Files

You can read from a text file using the `read()`, `readline()`, or `readlines()` methods, and you can write using the `write()` or `writelines()` methods.

```python
# Writing to a file
f = open("test.txt", "w")
f.write("Hello, World!")
f.close()

# Reading from a file
f = open("test.txt", "r")
content = f.read()
f.close()

print(content)  # Prints "Hello, World!"
```

---

### Reading and Writing Binary Files

You can read from and write to binary files just like text files, but you use the `'b'` mode.

```python
# Writing binary data to a file
data = bytes(range(5))
f = open("test.bin", "wb")
f.write(data)
f.close()

# Reading binary data from a file
f = open("test.bin", "rb")
data = f.read()
f.close()

print(list(data))  # Prints "[0, 1, 2, 3, 4]"
```

---

### The `with` Statement

Python provides the `with` statement to make working with files easier and cleaner. It automatically closes the file when you're done with it.

```python
with open("test.txt", "w") as f:
    f.write("Hello, World!")

with open("test.txt", "r") as f:
    print(f.read())  # Prints "Hello, World!"
```

---

## Summary

You've learned how to work with files in Python, including opening and closing files, reading from and writing to text and binary files, and using the `with` statement for better file handling.

---
