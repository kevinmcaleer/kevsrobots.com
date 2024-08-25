---
layout: lesson
title: Managing Software
author: Kevin McAleer
type: page
cover: /learn/linux_intro/assets/uninstall.jpg
date: 2024-08-23
previous: 06_installing_software.html
next: 08_monitoring_performance.html
description: Learn how to remove, update, and manage installed software using apt.
percent: 36
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

Managing installed software is just as important as installing it. This lesson will teach you how to remove, update, and manage software packages using the `apt` package manager.

---

## Learning Objectives

- Remove software packages using `apt remove` and `apt purge`.
- Update installed software using `apt upgrade`.
- Clean up unnecessary packages with `apt autoremove`.

---

### Removing Software

If you no longer need a software package, you can remove it using `apt remove`. For example:

        sudo apt remove git

To remove a package and its configuration files, use apt purge:

        sudo apt purge git

---

### Updating Installed Software

To update all installed packages to their latest versions, use:

        sudo apt upgrade

For a more thorough upgrade that handles dependencies better, use:

        sudo apt full-upgrade

## Cleaning Up Unnecessary Packages

Over time, unused packages can accumulate on your system. To remove packages that are no longer needed, use:

        sudo apt autoremove

---

## Summary

In this lesson, you learned how to manage installed software by removing, updating, and cleaning up packages using `apt`. Keeping your system clean and up-to-date is essential for maintaining performance and security.

---
