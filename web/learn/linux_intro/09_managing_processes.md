---
layout: lesson
title: Managing Processes
author: Kevin McAleer
type: page
cover: /learn/linux_intro/assets/kill.jpg
date: 2024-08-23
previous: 08_monitoring_performance.html
next: 10_networking_commands.html
description: Learn how to manage processes using commands like kill, killall, and
  how to work with background processes.
percent: 50
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

Processes are running instances of programs. Managing them is a key part of system administration. In this lesson, you'll learn how to manage processes, including how to kill unresponsive processes and how to work with background processes.

---

## Learning Objectives

- Understand what a process ID (PID) is.
- Kill processes using `kill` and `killall`.
- Work with background and foreground processes.

---

### Understanding Process IDs (PIDs)

Every running process has a unique identifier called a Process ID (PID). You can find the PID of a process using the `ps` or `top` commands:

        ps

    This will display a list of running processes along with their PIDs.

        PID TTY          TIME CMD
        2783 pts/0    00:00:00 bash
        2794 pts/0    00:00:00 ps

---

To see more detailed information about the processes, you can use:

        ps aux

This will show more detailed information about the processes.

```bash
USER         PID %CPU %MEM    VSZ   RSS TTY      STAT START   TIME COMMAND
root           1  0.0  0.5 169136 11168 ?        Ss   Aug23   0:00 /sbin/init sp
root           2  0.0  0.0      0     0 ?        S    Aug23   0:00 [kthreadd]
root           3  0.0  0.0      0     0 ?        S    Aug23   0:00 [pool_workque
root           4  0.0  0.0      0     0 ?        I<   Aug23   0:00 [kworker/R-rc
root           5  0.0  0.0      0     0 ?        I<   Aug23   0:00 [kworker/R-rc
root           6  0.0  0.0      0     0 ?        I<   Aug23   0:00 [kworker/R-sl
root           7  0.0  0.0      0     0 ?        I<   Aug23   0:00 [kworker/R-ne
root          11  0.0  0.0      0     0 ?        I    Aug23   0:00 [kworker/u8:0
```

---

`grep` is a command used to search for text patterns. You can use it to filter the output of `ps`:

        ps aux | grep <process-name>

Notice the `|` symbol, which is a pipe that sends the output of `ps aux` to `grep`.

---

## Killing Processes with kill and killall

`kill`: Sends a signal to a process to terminate it. You need the PID to use this command.

        kill <PID>

---

`killall`: Kills all processes with the specified name.

        killall <process-name>

---

## Background and Foreground Processes

`Background Processes`: Run a command in the background by adding & at the end.

        long_running_command &

---

`Foreground Processes`: Bring a background process to the foreground using fg.

        fg

---

`Pausing and Resuming`: Pause a process with `Ctrl + Z` and resume it with `bg` for background or `fg` for foreground.

---

## Summary

In this lesson, you learned how to manage processes by identifying PIDs, killing unresponsive processes, and working with background and foreground processes. Effective process management is crucial for maintaining system stability.

---
