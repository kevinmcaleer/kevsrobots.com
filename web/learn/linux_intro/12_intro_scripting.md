---
layout: lesson
title: Introduction to Shell Scripting
author: Kevin McAleer
type: page
cover: /learn/linux_intro/assets/shell.jpg
date: 2024-08-23
previous: 11_transferring_files.html
next: 13_basic_scripting_constructs.html
description: Learn the basics of shell scripting, including how to create and run
  simple scripts.
percent: 65
duration: 4
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

Shell scripting allows you to automate tasks on your Raspberry Pi by writing scripts—collections of commands executed in sequence. This lesson introduces you to the basics of shell scripting, including how to create and run simple scripts.

---

## Learning Objectives

- Understand what a shell script is and its uses.
- Create a simple shell script.
- Run and test your shell script.

---

### What is a Shell Script?

A `shell script` is a text file containing a sequence of commands that the shell (command interpreter) can execute. Shell scripts can automate repetitive tasks, manage system configurations, and much more.

---

### Creating a Simple Shell Script

`echo` is a command that prints text to the terminal.

      echo "Hello, World!"

This will simply print "Hello, World!" to the terminal.

You can also redirect the text to a file by using the `>` operator:

      echo "Hello, World!" > hello.sh

This will create a file named `hello.sh` that contains the text "Hello, World!".

How do we know the contents of the file? For that, we can use the `cat` command.

---

## What is the `cat` Command?

`cat` (catalog) is a command that displays the contents of a file:

   cat hello.sh

---

## How to Edit a File

`echo` and `cat` only works if you want to add a single line of text, or show the contents of a file. If you want to add multiple lines of text or edit an existing file you can use a text editor like `nano`.

---

To create a shell script, you can use the `nano` text editor. For example, to create a script that prints "Hello, World!":

1. Open `nano` and create a new file:

         nano hello.sh

   Add the following content to the file:

         #!/bin/bash
         echo "Hello, World!"

   Save and exit nano (use `Ctrl + O` to save and `Ctrl + X` to exit).

---

## Running Your Shell Script

To run your script, you need to make it executable and then execute it:

Make the script executable:

      chmod +x hello.sh

   This will give the script permission to run as an executable.

   Run the script:

      ./hello.sh

You should see "Hello, World!" printed in the terminal.

---

## Vim

If you prefer using `vim` as a text editor, you can create a shell script using the following steps:

1. Open `vim` and create a new file:

         vim hello.sh

1. Press `i` to enter insert mode and add the following content to the file:

         #!/bin/bash
         echo "Hello, World!"

1. Press `Esc` to exit insert mode, then type `:wq` and press `Enter` to save and exit.

> ## What does Vim stand for
>
> Vim stands for Vi IMproved.
>
> ---
>
> ### Explanation
>
> `Vi`: Vim is an enhanced version of the older text editor called `vi`, which stands for "visual." Vi was one of the earliest text editors in the Unix operating system and became widely used due to its efficiency and powerful editing capabilities.
>
`IMproved`: Vim stands for "Vi IMproved" because it includes many enhancements and additional features that were not present in the original vi editor. These improvements include features like syntax highlighting, multi-level undo, support for plugins, and a more extensive command set, making it a much more powerful and flexible tool than the original vi.
>
> `vim` retains compatibility with `vi` while offering these additional features, which is why it's often considered a superset of `vi`. Despite its steep learning curve, Vim is highly regarded for its speed, efficiency, and the extensive control it gives users over text editing.

---

## Summary

In this lesson, you learned what a shell script is and how to create and run a simple shell script. Shell scripting is a powerful tool for automating tasks and managing your Raspberry Pi more efficiently.

---
