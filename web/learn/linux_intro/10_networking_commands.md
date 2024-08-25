---
layout: lesson
title: Networking Commands
author: Kevin McAleer
type: page
cover: /learn/linux_intro/assets/ipconfig.jpg
date: 2024-08-23
previous: 09_managing_processes.html
next: 11_transferring_files.html
description: Learn how to check network status and test connectivity using commands
  like ifconfig, ip, ping, and traceroute.
percent: 48
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

Networking is a key aspect of using your Raspberry Pi, especially for projects that involve remote access or internet connectivity. In this lesson, you'll learn how to check your network status and test connectivity using essential networking commands.

---

## Learning Objectives

- Check network status using `ifconfig` and `ip`.
- Test network connectivity using `ping` and `traceroute`.
- Connect to another machine using `ssh`.

---

### Checking Network Status

- **`ifconfig`**: Displays network configuration details such as IP addresses and MAC addresses.

      ifconfig

- **`ip`**: A more modern and powerful tool for network management. To view your network interfaces, use:

      ip a

---

## Testing Network Connectivity

- **`ping`**: Tests the connection to another device on the network by sending ICMP echo requests.

      ping google.com

- **`traceroute`**: Displays the route that packets take to reach a destination.

      traceroute google.com

---

## Using SSH to Connect Remotely

The `ssh` (**Secure Shell**) command allows you to securely connect to another machine over a network. To connect to a Raspberry Pi with the IP address 192.168.1.10, use:

    ssh pi@192.168.1.10

You'll be prompted to enter the password for the `pi` user.

---

## Summary

In this lesson, you learned how to check network status, test connectivity, and connect to another machine using SSH. These commands are essential for managing and troubleshooting network-related tasks on your Raspberry Pi.

---
