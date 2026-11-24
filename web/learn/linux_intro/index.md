---
layout: course_overview
title: Introduction to the Linux Command Line on Raspberry Pi OS
author: Kevin McAleer
cover: /learn/linux_intro/assets/cover.jpg
date: 2024-08-23
description: Learn the fundamentals of using the Linux command line on Raspberry Pi
  OS. This course covers essential topics including navigation, file management, software
  installation, system monitoring, networking, and basic scripting.
level: beginner
course_duration: 2h 10m
lesson_count: 22
first_lesson: 00_intro.html
quiz_total: 85
video: UDo6tvoOQHg
outcomes:
- Move around the Raspberry Pi OS filesystem and manage files and folders confidently
- Read and change file permissions, users and groups
- Install, update and remove software with apt
- Check disk, memory, temperature, processes and services with systemctl
- Log in over SSH and copy files between your computer and the Pi
- Write a shell script and schedule it with cron, tracked with Git
prerequisites:
- None. This is the first course in the Homelab learning pathway
you_need:
- A Raspberry Pi 4 or 5 running Raspberry Pi OS (64-bit), with power, a microSD card
  and a network connection
- A keyboard and screen, or a second computer to log in from over SSH
navigation:
- name: Introduction to the Linux Command Line on Raspberry Pi OS
- content:
  - section: Getting Started with the Command Line
    content:
    - name: Course Overview
      link: 00_intro.html
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
    - name: Services and systemctl
      link: 09a_services.html
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
  - section: Summary and Next Steps
    content:
    - name: Summary and Next Steps
      link: 18_resources_next_steps.html
    - name: Linux Command Line Cheatsheet
      link: 19_cheatsheet.html
---
