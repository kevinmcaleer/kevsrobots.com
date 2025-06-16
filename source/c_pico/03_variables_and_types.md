---
title: Variables and Data Types
description: Learn how to store and manipulate data in C using variables and different data types.
layout: lesson
type: page
cover: assets/3.jpg
date_updated: 2025-06-15
---

![Cover]({{page.cover}}){:class="cover"}

---

To make your programs useful, you need to **store data**, work with numbers, and control logic.  
That’s where **variables** and **data types** come in.

In this lesson, we’ll explore how to declare variables in C and which data types are available.

---

## What Is a Variable?

A **variable** is a named piece of memory that stores a value.  
Think of it as a labeled box where you can keep something — like a number or a letter.

In C, you must:

1. Choose a **type** (what kind of data you want to store)
2. Choose a **name** for the variable (like `age` or `temperature`)
3. Optionally set an **initial value**

---

## Basic Data Types in C

| Type    | Description                   | Example Value |
|---------|-------------------------------|---------------|
| `int`   | Integer number                | `42`          |
| `float` | Decimal number (approx.)      | `3.14`        |
| `char`  | Single character              | `'A'`         |
| `bool`  | True/false (with `stdbool.h`) | `true`        |
{:class="table table-striped"}

---

## Declaring Variables

Here’s how to declare and use variables:

```c
#include <stdio.h>
#include <stdbool.h>  // needed for bool

int main() {
    int age = 25;
    float temperature = 23.5;
    char grade = 'B';
    bool isOn = true;

    printf("Age: %d\n", age);
    printf("Temperature: %.1f\n", temperature);
    printf("Grade: %c\n", grade);
    printf("Is it on? %d\n", isOn);

    return 0;
}
```

> Try it: Change the values and run the program again!

## Comments in C

Notice the `//` at the end of the line in the code above?

That's a comment! Comments are ignored by the compiler and are used to explain your code. In C, comments start with `//` for single-line comments or `/* ... */` for multi-line comments.

```c
// This is a single-line comment

/* This is a
   multi-line comment */
```

---

## Arithmetic in C

You can do math using these operators:

| Symbol | Meaning            | Example |
| ------ | ------------------ | ------- |
| `+`    | Add                | `a + b` |
| `-`    | Subtract           | `a - b` |
| `*`    | Multiply           | `a * b` |
| `/`    | Divide             | `a / b` |
| `%`    | Modulo (remainder) | `a % b` |
{:class="table table-striped"}

---

## C is a Typed Language

In C, the type of a variable cannot change once it’s declared.

```c
int age = 30;
age = "thirty";  // ❌ Error: incompatible type
```

> **Note:** You must declare a variable **before** using it.

---

## Summary

* Variables let you store and use values in your programs
* C requires you to define **data types** (int, float, char, etc.)
* You can do math and print values using `printf`

---

Next up: [Conditionals and Loops](04_conditionals_and_loops) — where we’ll teach your program to make decisions and repeat things!

---
