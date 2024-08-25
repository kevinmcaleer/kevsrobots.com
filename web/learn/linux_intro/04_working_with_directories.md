---
layout: lesson
title: Working with Directories
author: Kevin McAleer
type: page
cover: /learn/linux_intro/assets/directories.png
date: 2024-08-23
previous: 03_working_with_files.html
next: 05_file_permissions.html
description: Learn how to create, remove, copy, and move directories using the command
  line.
percent: 24
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

Directories help you organize files on your Raspberry Pi. In this lesson, you'll learn how to create, remove, copy, and move directories using the command line.

---

## Learning Objectives

- List directories using `vdir`.
- Create directories using `mkdir`.
- Remove files and directories using `rm` and `rmdir`.
- Copy and move files and directories using `cp` and `mv`.

---

## Working with Directories

### Listing Directories

The `vdir` command is used to ***list*** directories. For example, to list the contents of the current directory, type:

```bash
vdir
```

This will display a list of files and directories in the current directory. You can also use `ls` as an alias for `vdir`.

---

### Creating and Removing Directories

- **Create a new directory**: Use the `mkdir` command to ***create*** a new directory.

    For example, to create a directory named `Projects`, type:

        mkdir Projects

- **Removing Directories**: Use `rmdir` to ***remove*** an empty directory.

        rmdir Projects

- **Removing Non-Empty Directories**: Use rm -r to remove a directory and its contents.

        rm -r Projects

---

### Copying and Moving Files and Directories

- **Copying**: Use `cp` to ***copy*** files and directories.

        cp example.txt example_copy.txt
        cp -r Projects Projects_backup

- **Moving**: Use `mv` to ***move*** or ***rename*** files and directories.

        mv example.txt Documents/
        mv Projects MyProjects

- **Removing Files**: Use `rm` to delete files.

        rm example.txt

---

## What do the commands mean?

Command | Description
--------|--------------------------------
`mkdir` | ***make directory***
`rmdir` | ***remove directory***
`rm -r` | ***remove recursively***
`cp`    | ***copy***
`mv`    | ***move***
`rm`    | ***remove***
`vdir`  | ***verbose directory listing***
{:class="table"}

---

## Summary

In this lesson, you learned how to ***list***, ***create***, ***remove***, ***copy***, and ***move*** directories. These commands help you keep your filesystem organized and efficient.

---
