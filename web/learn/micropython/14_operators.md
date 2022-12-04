---
layout: lesson
title: Operators
author: Kevin McAleer
type: page
previous: 13_loops.html
next: 15_functions.html
description: Comparitive operators
percent: 80
duration: 14
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

## Operators

---

Operators are the symbols such as `+` and `-`.

There are **many** types of operators, and we use them to do things like add numbers together, compare numbers and variables, and do some more advanced things that python is good at, like managing `sets`. We'll look at those in later tutorials.

Here is a list of all the operators available. Don't worry if this is overwhelming, you don't need to know many, and you probably already know a few of the [arithmetic ones](#arithmetic-operators).

Have a look through these, but don't worry, there isn't a test!

---

Here are the shortcut links to each of the types of Operators defined below:

[Arithmetic Operators](#arithmetic-operators){:.btn.btn-primary.my-3}
[Comparison (Relational) Operators](#comparison-operators){:.btn.btn-primary.my-3}
[Assignment Operators](#assignment-operators){:.btn.btn-primary.my-3}
[Logical Operators](#logical-operators){:.btn.btn-primary.my-3}
[Bitwise Operators](#bitwise-operators){:.btn.btn-primary.my-3}
[Membership Operators](#membership-operators){:.btn.btn-primary.my-3}
[Identity Operators](#identity-operators){:.btn.btn-primary.my-3}

---

## Arithmetic Operators

In the examples below `a` = `10`, `b` = `20`.

Table of MicroPython Arithmetic Operators
{:.caption}

| Operator       | Symbol | Description                                                                                                                                                                                                                                                 | Example                                                           |
|----------------|:------:|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------|
| Addition       |  `+`   | Adds values on either side of the operator.                                                                                                                                                                                                                 | `a + b = 30`                                                      |
| Subtraction    |  `-`   | Subtracts right hand operand from left hand operand.                                                                                                                                                                                                        | `a – b = -10`                                                     |
| Multiplication |  `*`   | Multiplies values on either side of the operator                                                                                                                                                                                                            | `a * b = 200`                                                     |
| Division       |  `/`   | Divides left hand operand by right hand operand                                                                                                                                                                                                             | `b / a = 2.0`                                                     |
| Modulus        |  `%`   | Divides left hand operand by right hand operand and returns remainder                                                                                                                                                                                       | `b % a = 0`                                                       |
| Exponent       |  `**`  | Performs exponential (power) calculation on operators                                                                                                                                                                                                       | `a**b =10` to the power 20                                        |
| Floor          |  `//`  | Floor Division - The division of operands where the result is the quotient in which the digits after the decimal point are removed. But if one of the operands is negative, the result is floored, i.e., rounded away from zero (towards negative infinity) | `9//2 = 4` and `9.0//2.0 = 4.0`, `-11//3 = -4`, `-11.0//3 = -4.0` |
{:class="table-w100 table table-bordered"}

---

## Comparison Operators

Table of MicroPython Comparitive Operators
{:.caption}

| Operator              | Symbol | Description                                                                                                       | Example                                               |
|-----------------------|:------:|-------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------|
| Equals                |  `==`  | If the values of two operands are equal, then the condition becomes true.                                         | `(a == b)` is not true.                               |
| Not Equal             |  `!=`  | If values of two operands are not equal, then condition becomes true.                                             | `(a != b)` is true.                                   |
| Not Equal             |  `<>`  | If values of two operands are not equal, then condition becomes true.                                             | `(a <> b)` is true. This is similar to `!=` operator. |
| Greater than          |  `>`   | If the value of left operand is greater than the value of right operand, then condition becomes true.             | `(a > b)` is not true.                                |
| Less than             |  `<`   | If the value of left operand is less than the value of right operand, then condition becomes true.                | `(a < b)` is true.                                    |
| Greater or Equal to   |  `>=`  | If the value of left operand is greater than or equal to the value of right operand, then condition becomes true. | `(a >= b)` is not true.                               |
| Less than or Equal to |  `<=`  | If the value of left operand is less than or equal to the value of right operand, then condition becomes true.    | `(a <= b)` is true.                                   |
{:class="table-w100 table table-bordered"}

---

## Assignment Operators

Table of MicroPython Assignment Operators
{:.caption}

| Operator       | Symbol | Description                                                                                | Example                                 |
|----------------|:------:|--------------------------------------------------------------------------------------------|-----------------------------------------|
| Equals         |  `=`   | Assigns values from right side operands to left side operand                               | `a = 10` assigns value of `10` into `a` |
| Add AND        |  `+=`  | It adds right operand to the left operand and assign the result to left operand            | `c += a` is equivalent to `c = c + a`   |
| Subtract AND   |  `-=`  | It subtracts right operand from the left operand and assign the result to left operand     | `c -= a` is equivalent to `c = c - a`   |
| Multiply AND   |  `*=`  | It multiplies right operand with the left operand and assign the result to left operand    | `c *= a` is equivalent to `c = c * a`   |
| Divide AND     |  `/=`  | It divides left operand with the right operand and assign the result to left operand       | `c /= a` is equivalent to `c = c / a`   |
| Modulus AND    |  `%=`  | It takes modulus using two operands and assign the result to left operand                  | `c %= a` is equivalent to `c = c % a`   |
| Exponent AND   | `**=`  | Performs exponential (power) calculation on operators and assign value to the left operand | `c **= a` is equivalent to `c = c ** a` |
| Floor Division | `//=`  | It performs floor division on operators and assign value to the left operand               | `c //= a` is equivalent to `c = c // a` |
{:class="table-w100 table table-bordered"}

---

## Logical Operators

Table of MicroPython Logical Operators
{:.caption}

| Operator    | Symbol | Description                                                          | Example                  |
|-------------|:------:|----------------------------------------------------------------------|--------------------------|
| Logical AND | `and`  | If both the operands are true then condition becomes true.           | `(a and b)` is true.     |
| Logical OR  |  `or`  | If any of the two operands are non-zero then condition becomes true. | `(a or b)` is true.      |
| Logical NOT | `not`  | Used to reverse the logical state of its operand.                    | `not(a and b)` is false. |
{:class="table-w100 table table-bordered"}

---

## Bitwise Operators

In the examples below `a` = `60`, `b` = `13`.

Table of MicroPython Bitwise Operators
{:.caption}

| Operator               | Symbol | Description                                                                                  | Example                                                                              |
|------------------------|:------:|----------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------|
| Binary AND             |  `&`   | Operator copies a bit to the result if it exists in both operands                            | `(a & b) = 12` (means 0000 1100)                                                          |
| Binary OR              |  `|`   | It copies a bit if it exists in either operand.                                              | `(a | b) = 61` (means 0011 1101)                                                     |
| Binary XOR             |  `^`   | It copies the bit if it is set in one operand but not both.                                  | `(a ^ b) = 49` (means 0011 0001)                                                     |
| Binary Ones Complement |  `~`   | It is unary and has the effect of 'flipping' bits.                                           | `(~a ) = -61` (means 1100 0011 in 2's complement form due to a signed binary number. |
| Binary Left Shift      |  `<<`  | The left operands value is moved left by the number of bits specified by the right operand.  | `a << 2 = 240` (means 1111 0000)                                                     |
| Binary Right Shift     |  `>>`  | The left operands value is moved right by the number of bits specified by the right operand. | `a >> 2 = 15` (means 0000 1111)                                                      |
{:class="table-w100 table table-bordered"}

---

## Membership Operators

Table of MicroPython Membership Operators
{:.caption}

| Operator | Description                                                                                      | Example                                                                      |
|:--------:|--------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------|
|   `in`   | Evaluates to true if it finds a variable in the specified sequence and false otherwise.          | `x in y`, here in results in a 1 if x is a member of sequence y.             |
| `not in` | Evaluates to true if it does not finds a variable in the specified sequence and false otherwise. | `x not in y`, here not in results in a 1 if x is not a member of sequence y. |
{:class="table-w100 table table-bordered"}

---

## Identity Operators

Table of MicroPython Identity Operators
{:.caption}

| Operator | Description                                                                                                     | Example                                                                  |
|:--------:|-----------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------|
|   `is`   | Evaluates to true if the variables on either side of the operator point to the same object and false otherwise. | `x is y`, here `is` results in 1 if id(x) equals id(y).                  |
| `is not` | Evaluates to false if the variables on either side of the operator point to the same object and true otherwise. | `x is not y`, here `is not` results in 1 if id(x) is not equal to id(y). |
{:class="table-w100 table table-bordered"}

---