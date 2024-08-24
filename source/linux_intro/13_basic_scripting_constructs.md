---
title: Basic Scripting Constructs
description: Learn how to use conditional statements, loops, and variables in your shell scripts.
layout: lesson
cover: /learn/linux_intro/assets/code.jpg
---

![cover image]({{page.cover}}){:class="cover"}

## Introduction

Shell scripts become more powerful when you use programming constructs like conditional statements, loops, and variables. In this lesson, you'll learn how to use these basic scripting constructs to make your scripts more dynamic and functional.

---

## Learning Objectives

- Use variables in shell scripts.
- Implement conditional statements (`if`, `else`).
- Use loops (`for`, `while`) to repeat tasks.

---

### Using Variables in Shell Scripts

Variables allow you to store and reuse values in your scripts. To define a variable and use it:

        #!/bin/bash
        name="Raspberry Pi"
        echo "Hello, $name!"

Note that the variable `name` should not have spaces around the `=` sign. Also note when the variable is used, it is prefixed with a `$`; `$name` in this case.

---

## Conditional Statements

Conditional statements enable your script to make decisions. Here's a basic example using if and else:

        #!/bin/bash
        if [ -f "/etc/passwd" ]; then
            echo "The file exists."
        else
            echo "The file does not exist."
        fi

---

## Using Loops

Loops allow you to repeat tasks multiple times. Here’s an example of a for loop:

        #!/bin/bash
        for i in 1 2 3 4 5
        do
            echo "Iteration $i"
        done

And a while loop:

        #!/bin/bash
        count=1
        while [ $count -le 5 ]
        do
            echo "Count is $count"
            count=$((count + 1))
        done

---

## Summary

In this lesson, you learned how to use variables, conditional statements, and loops in your shell scripts. These constructs make your scripts more powerful and flexible, allowing you to automate complex tasks more effectively.

---
