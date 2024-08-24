---
layout: lesson
title: Transferring Files Over a Network
author: Kevin McAleer
type: page
cover: /learn/linux_intro/assets/filecopy.jpg
date: 2024-08-23
previous: 10_networking_commands.html
next: 12_intro_scripting.html
description: Learn how to securely transfer files between machines using scp and rsync.
percent: 60
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

Transferring files over a network is a common task, especially when working with multiple devices. In this lesson, you'll learn how to securely transfer files between machines using `scp` and `rsync`.

---

## Learning Objectives

- Transfer files securely between machines using `scp`.
- Use `rsync` for efficient file transfers and synchronization.

---

### Transferring Files with `scp`

The `scp` (secure copy) command is used to transfer files between two machines over a secure connection. For example, to copy a file from your Raspberry Pi to another machine:

    scp example.txt pi@192.168.1.10:/home/pi/

To copy a file from another machine to your Raspberry Pi:

    scp pi@192.168.1.10:/home/pi/example.txt /home/pi/

## Synchronizing Files with rsync

The `rsync` command is a powerful tool for efficiently transferring and synchronizing files between machines. It only transfers the differences between the source and the destination, making it faster than scp for large directories.

    rsync -avh /home/pi/Projects/ pi@192.168.1.10:/home/pi/Projects/

The -avh flags stand for archive mode, verbose output, and human-readable output, respectively.

---

## Summary

In this lesson, you learned how to securely transfer files between machines using `scp` and `rsync`. These tools are essential for managing files across multiple devices, especially in networked environments.

---
