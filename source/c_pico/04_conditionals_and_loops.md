---
title: Conditionals and Loops
description: Learn how to write programs that make decisions and repeat actions using if-statements, while-loops, and for-loops in C.
layout: lesson
type: page
cover: assets/pico-c-cover.jpg
date_updated: 2025-06-15
---

![Cover](assets/pico-c-cover.jpg){:class="cover"}

---

Now that you can store data with variables, let’s add _logic_ to your programs with **conditionals** and **loops**.

This lesson will teach your programs how to make decisions and do things over and over again — two key skills for any kind of automation or embedded programming.

---

## Conditionals (if-statements)

Use `if` to check if something is true, and act accordingly.

```c
#include <stdio.h>

int main() {
    int temperature = 30;   // Set the variable temperature to 30

    if (temperature > 25) { // Check if temperature is greater than 25
        printf("It's hot today!\n"); 
    } else {                // If not, do this
        printf("It's not too hot.\n");
    }

    return 0;               // End the program
}
```

You can also use `else if`:

```c
if (temp > 25) {
    // ...
} else if (temp > 15) {
    // ...
} else {
    // ...
}
```

---

## Comparison Operators

| Operator | Meaning          | Example  |
| -------- | ---------------- | -------- |
| `==`     | Equal to         | `a == b` |
| `!=`     | Not equal to     | `a != b` |
| `>`      | Greater than     | `a > b`  |
| `<`      | Less than        | `a < b`  |
| `>=`     | Greater or equal | `a >= b` |
| `<=`     | Less or equal    | `a <= b` |
{:class="table table-striped"}

---

## Loops

Loops let you repeat something multiple times — great for blinking LEDs, checking sensors, or running forever.

### `while` Loop

```c
int count = 0;

while (count < 5) { // Repeat the code block while count is less than 5
    printf("Count: %d\n", count);
    count++; // Increment count by 1
}
```

This will print `Count: 0` through `Count: 4`.

---

### `for` Loop

A `for` loop is a shorter way to repeat something a specific number of times. for loops require three parts:

1. **Initialization**: Set a starting point: `int i = 0;` this sets the starting value of the loop variable `i`.
2. **Condition**: Check if the loop should continue: `i < 5;` this checks if `i` is less than 5, when the value of `i` reaches 5, the loop stops.
3. **Increment**: Update the loop variable after each iteration: `i++` this increases the value of `i` by 1 each time the loop runs.

```c
for (int i = 0; i < 5; i++) {
    printf("i = %d\n", i);
}
```

This does the same thing as the while loop above — it’s just shorter.

---

### Infinite Loop

Microcontrollers often run in an **infinite loop**, doing tasks over and over:

```c
while (1) {
    // Do something forever
}
```

We’ll use this when we get to controlling the LED!

---

## Summary

You now know how to:

* Make decisions with `if`, `else if`, and `else`
* Repeat actions with `while` and `for` loops
* Use loops for continuous behavior — a key part of embedded programming

---

Next up: [Functions in C](05_functions), where we’ll start breaking our programs into reusable chunks!

---
