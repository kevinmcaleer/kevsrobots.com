---
layout: lesson
title: Basic Scripting Constructs
author: Kevin McAleer
type: page
cover: /learn/linux_intro/assets/code.jpg
date: 2024-08-23
previous: 12_intro_scripting.html
next: 14_intro_git.html
description: Learn how to use conditional statements, loops, and variables in your
  shell scripts.
percent: 70
duration: 2
navigation:
- name: Introduction to the Linux Command Line on Raspberry Pi OS
- content:
  - section: Getting Started with the Command Line
    content:
    - name: Introduction to the Terminal
      link: 01_intro_terminal.html
    - name: Getting Help with `man`
      link: 01a_man.html
    - name: Basic Command Line Navigation
      link: 02_basic_navigation.html
  - section: Managing Files and Directories
    content:
    - name: Working with Files
      link: 03_working_with_files.html
    - name: Working with Directories
      link: 04_working_with_directories.html
    - name: File Permissions and Ownership
      link: 05_file_permissions.html
  - section: Managing Software
    content:
    - name: Installing Software
      link: 06_installing_software.html
    - name: Managing Software
      link: 07_managing_software.html
  - section: System Monitoring and Management
    content:
    - name: Monitoring System Performance
      link: 08_monitoring_performance.html
    - name: Managing Processes
      link: 09_managing_processes.html
  - section: Networking Basics
    content:
    - name: Networking Commands
      link: 10_networking_commands.html
    - name: Transferring Files Over a Network
      link: 11_transferring_files.html
  - section: Scripting Basics
    content:
    - name: Introduction to Shell Scripting
      link: 12_intro_scripting.html
    - name: Basic Scripting Constructs
      link: 13_basic_scripting_constructs.html
  - section: Working with Git (Optional Advanced Topic)
    content:
    - name: Introduction to Git
      link: 14_intro_git.html
    - name: Basic Git Commands
      link: 15_basic_git_commands.html
  - section: 'Final Project: Building a Simple Command Line Application'
    content:
    - name: Final Project Overview
      link: 16_final_project_overview.html
    - name: Final Project Steps
      link: 17_final_project_steps.html
  - section: Resources and Next Steps
    content:
    - name: Resources and Next Steps
      link: 18_resources_next_steps.html
    - name: Linux Command Line Cheatsheet
      link: 19_cheatsheet.html
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
