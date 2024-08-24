---
layout: lesson
title: Monitoring System Performance
author: Kevin McAleer
type: page
cover: /learn/linux_intro/assets/performance.webp
date: 2024-08-23
previous: 07_managing_software.html
next: 09_managing_processes.html
description: Learn how to monitor system performance using commands like df, du, top,
  and free.
percent: 45
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

Monitoring system performance is crucial for keeping your Raspberry Pi running smoothly. This lesson will introduce you to commands that help you keep track of disk usage, CPU load, and memory usage.

---

## Learning Objectives

- Check disk usage with `df` and `du`.
- Monitor CPU and process activity with `top` and `htop`.
- View memory usage with `free`.

---

### Checking Disk Usage

- **`df` (disk free)**: Displays the amount of disk space used and available on all mounted filesystems.

        df -h

    **Note** `-h`: Displays output in human-readable format (e.g., KB, MB, GB).

- **`du` (disk usage)**: Estimates file space usage. Use it to check how much space a directory is using.

        du -sh /home/pi

    **Note** `-s`: Displays only a total for each argument. `-h`: Displays output in human-readable format.

---

## Monitoring Processes with `top` and `htop`

- **`top`**: Displays real-time information about running processes, including CPU and memory usage.

        top

- **`htop`**: A more user-friendly version of top with an easier-to-read interface. You may need to install it first:

        sudo apt install htop
        htop

---

## Checking Memory Usage with `free`

The free command displays the total, used, and available memory on your Raspberry Pi. For human-readable output, use:

        free -h

---

## Summary

In this lesson, you learned how to monitor system performance using commands like `df`, `du`, `top`, and `free`. Regular monitoring can help you detect and troubleshoot potential issues before they become serious problems.

---
