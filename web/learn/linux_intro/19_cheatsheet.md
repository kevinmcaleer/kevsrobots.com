---
layout: lesson
title: Linux Command Line Cheatsheet
author: Kevin McAleer
type: page
cover: /learn/linux_intro/assets/cheatsheet.jpg
date: 2024-08-23
previous: 18_resources_next_steps.html
description: A quick reference for commonly used Linux command line commands with
  descriptions and examples.
percent: 100
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

## **Linux Command Line Cheatsheet**

| **Command** | **Description** | **Example Usage** |
|-------------|-----------------|-------------------|
| `pwd`       | Print Working Directory: Displays the current directory you are in. | `pwd` |
| `ls`        | List: Lists files and directories in the current directory. | `ls` |
| `cd`        | Change Directory: Moves you to the specified directory. | `cd /home/pi` |
| `mkdir`     | Make Directory: Creates a new directory. | `mkdir new_folder` |
| `rmdir`     | Remove Directory: Removes an empty directory. | `rmdir empty_folder` |
| `rm -r`     | Remove: Deletes a directory and its contents recursively. | `rm -r unwanted_folder` |
| `cp`        | Copy: Copies files or directories. | `cp file.txt /home/pi/` |
| `mv`        | Move: Moves or renames files or directories. | `mv file.txt /home/pi/newfile.txt` |
| `touch`     | Creates an empty file or updates the timestamp of an existing file. | `touch newfile.txt` |
| `nano`      | Opens the Nano text editor to create or edit files. | `nano textfile.txt` |
| `cat`       | Concatenates and displays the content of a file. | `cat file.txt` |
| `less`      | Displays file content one screen at a time. | `less file.txt` |
| `head`      | Displays the first few lines of a file. | `head -n 10 file.txt` |
| `tail`      | Displays the last few lines of a file. | `tail -n 10 file.txt` |
| `chmod`     | Changes the permissions of a file or directory. | `chmod 755 script.sh` |
| `chown`     | Changes the ownership of a file or directory. | `chown user:group file.txt` |
| `sudo`      | Executes a command with superuser (root) privileges. | `sudo apt update` |
| `apt`       | Advanced Package Tool: Used for managing packages (install, update, remove). | `sudo apt install package_name` |
| `df`        | Displays disk space usage of filesystems. | `df -h` |
| `du`        | Displays disk usage of files and directories. | `du -sh *` |
| `free`      | Displays memory usage. | `free -h` |
| `top`       | Displays real-time system processes and resource usage. | `top` |
| `htop`      | An enhanced version of `top`, requires installation. | `htop` |
| `ps`        | Displays information about running processes. | `ps aux` |
| `kill`      | Terminates a process by its ID (PID). | `kill 1234` |
| `killall`   | Terminates all processes with a given name. | `killall firefox` |
| `ifconfig`  | Displays network configuration details. | `ifconfig` |
| `ping`      | Sends ICMP echo requests to test connectivity to another host. | `ping 8.8.8.8` |
| `scp`       | Secure Copy: Copies files over SSH. | `scp file.txt user@remote:/path` |
| `rsync`     | Synchronizes files and directories between two locations. | `rsync -avz source/ destination/` |
| `ssh`       | Secure Shell: Connects to another machine securely over the network. | `ssh user@hostname` |
| `crontab`   | Schedules commands to run at specified intervals. | `crontab -e` |
| `git`       | Version control system for tracking changes in source code. | `git clone https://github.com/repo.git` |
| `echo`      | Displays a line of text/string passed as an argument. | `echo "Hello, World!"` |
| `grep`      | Searches text using patterns. | `grep "search_term" file.txt` |
| `find`      | Searches for files and directories in a directory hierarchy. | `find /home/pi -name "*.txt"` |
| `tar`       | Archives multiple files into a single file (optionally compressing). | `tar -czvf archive.tar.gz folder/` |
| `unzip`     | Extracts compressed files from a ZIP archive. | `unzip archive.zip` |
| `history`   | Shows the command history list. | `history` |
| `man`       | Displays the manual page for a command. | `man ls` |
| `exit`      | Closes the terminal session. | `exit` |
| `raspi-config` | Raspberry Pi Configuration Tool for system settings. | `raspi-config` |
| `vdir`      | Alias for `ls` command. | `vdir` |

---
