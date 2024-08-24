---
layout: lesson
title: Final Project Overview
author: Kevin McAleer
type: page
cover: /learn/linux_intro/assets/project.jpg
date: 2024-08-23
previous: 15_basic_git_commands.html
next: 17_final_project_steps.html
description: Plan and outline a simple command-line tool as your final project.
percent: 85
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

As a culmination of this course, you will create a simple command-line tool that incorporates the skills you've learned. This lesson will help you plan and outline your project.

---

## Learning Objectives

- Understand the scope and requirements of the final project.
- Plan the structure and functionality of your command-line tool.
- Identify the Linux commands and scripting techniques you will use.

---

### Project Overview

The final project involves creating a command-line tool that performs a useful task, such as backing up files, automating a system process, or monitoring system performance.

---

### Project Planning

Before you start coding, plan your project by answering these questions:

1. **What is the purpose of your tool?**
1. **Which commands will your tool use?**
1. **Will your tool require user input?**
1. **How will you test your tool?**

Write down the answers and outline the structure of your script.

---

### Example Project: Backup Script

For example, you could create a script that backs up a directory to a specified location and logs the operation:

1. Use `rsync` to copy files.
1. Use a loop to iterate over multiple directories.
1. Log the results to a file with `echo`.

---

## Summary

In this lesson, you planned and outlined your final project. Taking the time to plan your script will make the coding process smoother and help you create a more functional and efficient tool.

---
