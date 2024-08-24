---
title: Working with Directories
description: Learn how to create, remove, copy, and move directories using the command line.
layout: lesson
cover: /learn/linux_intro/assets/directories.png
---

![cover image]({{page.cover}}){:class="cover"}

## Introduction

Directories help you organize files on your Raspberry Pi. In this lesson, you'll learn how to create, remove, copy, and move directories using the command line.

---

## Learning Objectives

- List directories using `vdir`.
- Create directories using `mkdir`.
- Remove files and directories using `rm` and `rmdir`.
- Copy and move files and directories using `cp` and `mv`.

---

## Working with Directories

### Listing Directories

The `vdir` command is used to ***list*** directories. For example, to list the contents of the current directory, type:

```bash
vdir
```

This will display a list of files and directories in the current directory. You can also use `ls` as an alias for `vdir`.

---

### Creating and Removing Directories

- **Create a new directory**: Use the `mkdir` command to ***create*** a new directory.

    For example, to create a directory named `Projects`, type:

        mkdir Projects

- **Removing Directories**: Use `rmdir` to ***remove*** an empty directory.

        rmdir Projects

- **Removing Non-Empty Directories**: Use rm -r to remove a directory and its contents.

        rm -r Projects

---

### Copying and Moving Files and Directories

- **Copying**: Use `cp` to ***copy*** files and directories.

        cp example.txt example_copy.txt
        cp -r Projects Projects_backup

- **Moving**: Use `mv` to ***move*** or ***rename*** files and directories.

        mv example.txt Documents/
        mv Projects MyProjects

- **Removing Files**: Use `rm` to delete files.

        rm example.txt

---

## What do the commands mean?

Command | Description
--------|--------------------------------
`mkdir` | ***make directory***
`rmdir` | ***remove directory***
`rm -r` | ***remove recursively***
`cp`    | ***copy***
`mv`    | ***move***
`rm`    | ***remove***
`vdir`  | ***verbose directory listing***
{:class="table"}

---

## Summary

In this lesson, you learned how to ***list***, ***create***, ***remove***, ***copy***, and ***move*** directories. These commands help you keep your filesystem organized and efficient.

---
