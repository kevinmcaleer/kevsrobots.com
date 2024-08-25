---
layout: lesson
title: Working with Files
author: Kevin McAleer
type: page
cover: /learn/linux_intro/assets/files.png
date: 2024-08-23
previous: 02a_user.html
next: 04_working_with_directories.html
description: Learn how to create, view, edit, and manage files using the command line.
percent: 20
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
    - name: Managing Users and Using `sudo`
      link: 02a_user.html
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

Files are at the heart of everything you do on your Raspberry Pi. This lesson will teach you how to create, view, edit, and manage files using various command-line tools.

---

## Learning Objectives

- Create files using the `touch` command.
- View file contents with commands like `cat`, `less`, `head`, and `tail`.
- Edit files using the `nano` text editor.

---

### Creating Files with `touch`

The `touch` command is used to create an empty file. For example, to create a file named `example.txt`, you would use:

```bash
touch example.txt
```

A new file called `example.txt` will be created in the current directory, you can confirm this by using the `ls` command and checking for the file.

![touch command](/learn/linux_intro/assets/touch.png){:class="w-100 img-fluid card-hover card-shadow rounded-3"}

---

## Viewing File Contents

- `cat`: Concatenates and displays the contents of a file.

        cat example.txt

- `less`: Views file content page by page, useful for large files.

        less example.txt

- `head`: Displays the first few lines of a file.

        head example.txt

- `tail`: Displays the last few lines of a file.

        tail example.txt

---

## Editing Files with nano

The nano editor is a simple, easy-to-use text editor. To edit example.txt, type:

```bash
nano example.txt
```

Within `nano`, you can type text, and use keyboard shortcuts to save and exit (e.g., `Ctrl + O` to save and `Ctrl + X` to exit).

> We'll cover `nano` in more detail in a later lesson.

---

## Summary

In this lesson, you learned how to create, view, and edit files using the command line. Mastering these commands is crucial for managing files on your Raspberry Pi efficiently.

---
