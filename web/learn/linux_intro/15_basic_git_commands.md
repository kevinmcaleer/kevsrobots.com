---
layout: lesson
title: Basic Git Commands
author: Kevin McAleer
type: page
cover: /learn/linux_intro/assets/git.jpg
date: 2024-08-23
previous: 14_intro_git.html
next: 16_final_project_overview.html
description: Learn essential Git commands like init, add, commit, and clone to manage
  your repositories.
percent: 80
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

Git provides a range of commands to help you manage your repositories and track changes. In this lesson, you’ll learn the essential Git commands needed to start managing your projects.

---

## Learning Objectives

- Initialize a new Git repository.
- Add and commit changes to the repository.
- Clone an existing repository.

---

### Initializing a Git Repository

To start tracking a project with Git, navigate to the project directory and initialize a repository:

```bash
git init
```

---

## Adding and Committing Changes

After making changes to your files, you need to add them to the staging area and commit them:

```bash
git add .
git commit -m "Initial commit"
```

The add command stages your changes, and commit saves them to the repository with a descriptive message.

---

## Cloning a Repository

If you want to work on an existing project, you can clone its repository:

        git clone https://github.com/user/repo.git

This command downloads the project and its entire history to your local machine.

---

## Summary

In this lesson, you learned how to initialize a Git repository, add and commit changes, and clone existing repositories. These basic commands are the foundation for managing your projects with Git.

---
