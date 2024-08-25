---
layout: lesson
title: Introduction to Git
author: Kevin McAleer
type: page
cover: /learn/linux_intro/assets/git.jpg
date: 2024-08-23
previous: 13_basic_scripting_constructs.html
next: 15_basic_git_commands.html
description: Learn the basics of Git, a version control system, and how to set it
  up on Raspberry Pi OS.
percent: 64
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

`git` is a powerful version control system that helps you manage changes to your code over time. This lesson will introduce you to Git and show you how to set it up on your Raspberry Pi.

---

## Learning Objectives

- Understand what Git is and why it's useful.
- Install Git on Raspberry Pi OS.
- Configure Git with your user information.

---

### What is Git?

Git is a distributed version control system that tracks changes to files and allows multiple people to collaborate on a project. It’s widely used in software development but is also useful for managing any kind of file.

---

### Installing Git on Raspberry Pi OS

To install Git on your Raspberry Pi, use the `apt` package manager:

        sudo apt update
        sudo apt install git

---

## Configuring Git

After installing Git, you need to configure it with your name and email address. These details will be used in your commits:

        git config --global user.name "Your Name"
        git config --global user.email "your.email@example.com"

---

## Summary

In this lesson, you learned what Git is, how to install it on Raspberry Pi OS, and how to configure it with your user information. Git is an essential tool for managing your projects, especially as they grow in complexity.

---
