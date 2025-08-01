---
title: Functions in C
description: Learn how to organize and reuse code using functions in the C programming language.
layout: lesson
type: page
cover: assets/5.jpg
date_updated: 2025-06-15
---

![Cover]({{page.cover}}){:class="cover"}

---

As your programs grow, it becomes helpful to **organize your code into functions**.  
Functions let you name a **block of code** and call it whenever you need it — this keeps your code clean, reusable, and easier to understand.

---

## What is a Function?

A function is like a mini-program you can define and reuse.  
Every C program starts with a special function called `main()` — that’s where the program begins.

Here’s a simple function:

```c
void greet() {
    printf("Hello from a function!\n");
}
```

This function uses the `printf()` function to print a message. Functions can contain other functions, variables, and logic just like the main program.

The `\n` at the end of the string is a newline character, which moves the cursor to the next line after printing.

To run the function, just call it by name:

```c
int main() {
    greet();  // this runs the greet() function
    return 0;
}
```

---

## Function Syntax

`Syntax` means the rules for how to write code correctly.

Here’s the basic structure of a function in C:

```c
return_type function_name(parameter_list) {
    // code to execute
}
```

The parts are:

- **return_type**: What type of value the function returns (like `int`, `void`, etc.)
- **function_name**: The name you give to the function (like `greet`)
- **parameter_list**: Inputs the function can take (like `int a, int b`)
- **code to execute**: The block of code that runs when the function is called

---

### Example with parameters

```c
void say_hello(char name[]) {
    printf("Hello, %s!\n", name);
}
```

the `char name[]` part is a parameter that lets you pass in a name when you call the function. The square brackets `[]` indicate that `name` is an list of characters (a string).

Call it like this:

```c
say_hello("Kevin");
```

---

## Functions That Return Values

```c
int add(int a, int b) {
    return a + b;
}

int main() {
    int result = add(2, 3);
    printf("Result: %d\n", result);
    return 0;
}
```

---

## Why Use Functions?

* **Reusability** – write once, use many times
* **Readability** – break big problems into smaller steps
* **Maintainability** – change code in one place instead of many

---

> ## Quick Tips
>
> - Declare functions **before** calling them (or use a *function prototype*)
> - Always match the **number and type of arguments**
> - `void` means “no return value”

---

## Summary

You now know how to:

* Write and call your own functions
* Pass data into functions with parameters
* Return results from functions
* Use functions to clean up and simplify your code

---

Next up: [Setting Up the Toolchain](06_setting_up_toolchain), where we’ll install the software you need to compile and upload C programs to the Raspberry Pi Pico.

---
