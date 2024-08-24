---
layout: lesson
title: Getting Help with `man`
author: Kevin McAleer
type: page
cover: /learn/linux_intro/assets/man.png
date: 2024-08-23
previous: 01_intro_terminal.html
next: 02_basic_navigation.html
description: Learn how to use the `man` command to access manual pages for other commands,
  providing detailed information and usage examples.
percent: 10
duration: 3
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

As you start working with the Linux command line, you might come across commands or options you're not familiar with. The `man` command is an invaluable tool that provides detailed documentation for almost every command available on the system. In this lesson, you'll learn how to use `man` to access these manual pages, helping you understand and utilize commands effectively.

---

## Learning Objectives

- Understand the purpose of the `man` command.
- Learn how to access and navigate manual pages using `man`.
- Interpret the information provided in manual pages.

---

### What is the `man` Command?

The `man` command, short for "manual," provides a comprehensive reference for most commands and utilities in Linux. Each command typically has an associated manual page (or "man page") that explains what the command does, how to use it, and what options are available.

---

### Using the `man` Command

To use the `man` command, simply type `man` followed by the name of the command you want to learn more about. For example, to read the manual page for the `ls` command:

```bash
man ls
```

This will open the manual page for `ls`, displaying detailed information on how to use it.

![Man command](/learn/linux_intro/assets/man.png){:class="w-100 img-fluid card-hover card-shadow rounded-3"}

---

### Navigating the Manual Pages

Once inside a manual page, you can navigate using the following keys:

- **Arrow Keys**: Move up and down line by line.
- **Spacebar**: Move down one screen at a time.
- **b**: Move up one screen at a time.
- **/search_term**: Search for a specific term within the page (press `n` to jump to the next occurrence).
- **q**: Quit the manual page and return to the command prompt.

---

### Understanding the Structure of a Man Page

Most man pages follow a similar structure, typically including the following sections:

1. **NAME**: The name of the command and a brief description.
1. **SYNOPSIS**: A summary of how to use the command, showing the command’s syntax, including options and arguments.
1. **DESCRIPTION**: A detailed explanation of what the command does.
1. **OPTIONS**: Lists all the options available with the command, often including examples.
1. **EXAMPLES**: Practical examples of how to use the command.
1. **SEE ALSO**: References to related commands or further reading.

For example, in the `ls` man page, the OPTIONS section would describe flags like `-l` (long listing format) and `-a` (include hidden files).

---

### Finding the Right Man Page

Sometimes, you might not know the exact command but want to search for a keyword. You can do this using:

```bash
man -k search_term
```

This command searches the short descriptions in all the man pages for the keyword you provide and returns a list of relevant commands. For example:

```bash
man -k directory
```

This might return commands like `mkdir`, `rmdir`, and `ls`, which all relate to directories.

---

## Summary

In this lesson, you learned how to use the `man` command to access detailed documentation for Linux commands. Understanding how to navigate and interpret man pages is crucial for mastering the command line and solving problems independently.

---

## Practice Exercise

Try using the `man` command to learn more about the following commands:

1. `man pwd`
1. `man mkdir`
1. `man chmod`

Explore the options and examples provided in each man page to deepen your understanding of these commands.

---
