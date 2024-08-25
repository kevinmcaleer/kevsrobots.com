---
layout: lesson
title: Managing Users and Using `sudo`
author: Kevin McAleer
type: page
cover: /learn/linux_intro/assets/usermgr.jpg
date: 2024-08-23
previous: 02_basic_navigation.html
next: 03_working_with_files.html
description: Learn how to manage users, use the `sudo` command, and change passwords
  in a Unix-like system.
percent: 16
duration: 4
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

In a Unix-like operating system, managing users and permissions is a critical task, especially when multiple people use the same system. This lesson covers how to add and manage users, use the `sudo` command to execute commands with superuser privileges, and change user passwords.

---

## Learning Objectives

- Understand what the `sudo` command does and how to use it.
- Learn how to add, delete, and manage users.
- Change user passwords using the `passwd` command.

---

### Understanding `sudo`

The `sudo` command, short for "**superuser do**," allows a permitted user to execute a command as the superuser (or another user), as specified by the security policy. The superuser, also known as "root," has unrestricted access to the system, making `sudo` a powerful and necessary tool for performing administrative tasks.

#### **Using `sudo`:**

To use `sudo`, simply prepend it to the command you want to run with elevated privileges. For example:

```bash
sudo apt update
```

This command updates the package list for the system, but since it requires superuser privileges, `sudo` is used. You will usually be prompted to enter your password when using `sudo` to verify your identity.

#### **Why Use `sudo` Instead of Logging in as Root?**

- **Safety:** Using `sudo` reduces the risk of making critical errors because commands are run as superuser only when necessary.
- **Security:** It allows for better auditing of commands executed with superuser privileges, as each command is associated with the user who ran it.
- **Control:** Admins can control which users can execute which commands by configuring the `sudoers` file.

---

### Adding and Managing Users

#### **1. Adding a New User:**

To add a new user to the system, use the `adduser` command:

```bash
sudo adduser username
```

You will be prompted to enter and confirm a password for the new user, as well as provide some additional information like the full name (which is optional).

#### **2. Deleting a User:**

To delete a user and optionally remove their home directory, use the `deluser` command:

```bash
sudo deluser username
```

If you also want to remove the user’s home directory, use:

```bash
sudo deluser --remove-home username
```

#### **3. Adding a User to the `sudo` Group:**

If you want a user to have the ability to use `sudo`, you need to add them to the `sudo` group:

```bash
sudo usermod -aG sudo username
```

The `-aG` option appends the user to the group without removing them from any other groups they belong to.

---

### Changing Passwords

#### **1. Changing Your Own Password:**

To change your own password, use the `passwd` command:

```bash
passwd
```

You will be prompted to enter your current password followed by the new password twice.

#### **2. Changing Another User’s Password:**

If you have `sudo` privileges, you can change another user’s password:

```bash
sudo passwd username
```

After entering your own password (for `sudo`), you will be prompted to enter a new password for the specified user.

#### **3. Forcing a Password Change on Next Login:**

To force a user to change their password the next time they log in, use:

```bash
sudo passwd -e username
```

This command expires the user's password, requiring them to update it upon their next login.

---

### Managing Groups

Groups are a way to manage permissions for multiple users at once. You can add or remove users from groups to grant or revoke permissions.

- **Create a New Group:**
  ```bash
  sudo groupadd groupname
  ```

- **Add a User to a Group:**
  ```bash
  sudo usermod -aG groupname username
  ```

- **Remove a User from a Group:**
  ```bash
  sudo deluser username groupname
  ```

---

## Summary

In this lesson, you learned how to use the `sudo` command to execute commands with superuser privileges, manage users and groups, and change passwords. These skills are essential for securely administering a Unix-like system, especially when multiple users are involved.

---

## Practice Exercise

Try the following tasks to practice what you've learned:

1. **Add a new user:** Create a user called `student` and give them `sudo` privileges.
2. **Change your password:** Update your current password using the `passwd` command.
3. **Manage groups:** Create a new group called `developers` and add the `student` user to this group.

---
