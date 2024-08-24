---
layout: lesson
title: File Permissions and Ownership
author: Kevin McAleer
type: page
cover: /learn/linux_intro/assets/permissions_cover.jpg
date: 2024-08-23
previous: 04_working_with_directories.html
next: 06_installing_software.html
description: Understand file permissions and learn how to change them using chmod
  and chown.
percent: 30
duration: 5
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

File permissions and ownership are critical for system security and multi-user environments. This lesson will teach you how to view and modify file permissions and ownership.

---

## Learning Objectives

- Understand file permissions and how they work.
- Change file permissions using `chmod`.
- Change file ownership using `chown`.

---

### Understanding File Permissions

Every file and directory in Linux has a set of permissions that determine who can read, write, or execute it. These are represented by a series of letters and dashes, like `-rwxr-xr-x`.

- **r**: Read
- **w**: Write
- **x**: Execute

Permissions are divided into three groups:

1. **Owner**: The user who owns the file.
1. **Group**: The group that owns the file.
1. **Others**: Everyone else.

![permissions meaning](/learn/linux_intro/assets/permissions_meaning.png){:class="w-100 img-fluid card-hover card-shadow rounded-3"}

---

### **Understanding File and Directory Permissions in Unix**

In Unix-like operating systems (like Linux), every file and directory has a set of permissions that determine who can read, write, or execute them. These permissions control what different users can do with a file or directory.

#### **The Three Types of Permissions:**

1. **`r` - Read:**
   - If you have **read** permission (`r`), you can **open and view** the contents of a file or list the contents of a directory.
   - **Example:** If you have read permission on a text file, you can open and read it, but you can't make changes to it.

2. **`w` - Write:**
   - If you have **write** permission (`w`), you can **modify** the contents of a file or **make changes** within a directory.
   - **For a file:** This means you can edit the file, add new content, delete content, or even delete the file itself.
   - **For a directory:** Write permission lets you create new files or delete existing files within that directory.
   - **Example:** If you have write permission on a document, you can edit it, save changes, or even delete it.

3. **`x` - Execute:**
   - If you have **execute** permission (`x`), you can **run** the file as a program or script.
   - **For a file:** This is usually used for scripts or programs. If a file is marked as executable, you can run it directly from the command line.
   - **For a directory:** Execute permission means you can **enter** the directory (using the `cd` command) and access the files within it.
   - **Example:** If you have execute permission on a script file, you can run that script, and it will perform its programmed tasks.

#### **Who Can Have These Permissions?**

Permissions are set for three types of users:

1. **Owner (User):**
   - The person who created the file or directory. They usually have full control over it.

2. **Group:**
   - A group of users who are assigned certain permissions. This is useful for teams or collaborators working on the same files.

3. **Others:**
   - Everyone else who is not the owner or in the group. These are users who might need limited access to the file or directory.

#### **Example of Permissions:**

Let's say you have a file with the following permissions: `rwxr-xr--`.

- **Owner (`rwx`)**: The owner can read, write, and execute the file. This means they can do anything with the file—open it, edit it, run it as a program, etc.
- **Group (`r-x`)**: The group can read and execute the file, but they **cannot** modify it. They can view the file and run it if it's a script or program, but they can't change its contents.
- **Others (`r--`)**: Everyone else can only read the file. They can't modify or execute it.

#### **Putting It All Together:**

Permissions ensure that files and directories are secure and that only the right people can access or modify them. Understanding these basics will help you manage your files better and keep your system safe.

- **Read (`r`)**: You can **look at** the contents.
- **Write (`w`)**: You can **change or delete** the contents.
- **Execute (`x`)**: You can **run** the file as a program or **enter** the directory.

Learning to work with these permissions is an essential skill when using the command line, especially in multi-user environments like Linux servers or shared systems.

---

### Viewing Permissions

To view the permissions of a file, use the `ls -l` command:

        ls -l example.txt

---

### Changing Permissions with chmod

The `chmod` command is used to change file permissions. For example, to make a file executable by the owner, use:

        chmod u+x example.sh

the `u+x` means ***add execute permission for the owner***. You can also use `g` for group and `o` for others:

        chmod g+r example.sh
        chmod o-w example.sh

You can also use numeric values to set permissions:

        chmod 755 example.sh

The numeric values are calculated as follows:

- **4**: Read
- **2**: Write
- **1**: Execute

The sum of these values gives the permission level. For example, `755` means:

- **Owner**: Read, write, execute (4+2+1 = 7)
- **Group**: Read, execute (4+1 = 5)
- **Others**: Read, execute (4+1 = 5)

---

## Changing Ownership with chown

- The `chown` command changes the ownership of a file. For example, to change the owner of example.txt to pi:

        sudo chown pi example.txt

    In this example, `pi` is the new owner. 

- To change the group ownership:

        sudo chown :pi example.txt

    In this example, `pi` is the new group owner.

---

## What the commands mean

Command | Description
--------|--------------------------------
`chmod` | ***change file permissions*** - **ch**ange **mod**e
`chown` | ***change file ownership*** - **ch**ange **own**er

---

## Summary

In this lesson, you learned how to ***view*** and ***modify*** file permissions and ownership using the `chmod` and `chown` commands. Properly managing permissions is essential for securing your Raspberry Pi.

---
