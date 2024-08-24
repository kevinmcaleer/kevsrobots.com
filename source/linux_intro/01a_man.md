---
title: Getting Help with `man`
description: Learn how to use the `man` command to access manual pages for other commands, providing detailed information and usage examples.
layout: lesson
cover: /learn/linux_intro/assets/man.png
---

![cover image]({{page.cover}}){:class="cover"}

## Introduction

As you start working with the Linux command line, you might come across commands or options you're not familiar with. The `man` command is an invaluable tool that provides detailed documentation for almost every command available on the system. In this lesson, you'll learn how to use `man` to access these manual pages, helping you understand and utilize commands effectively.

---

## Learning Objectives

- Understand the purpose of the `man` command.
- Learn how to access and navigate manual pages using `man`.
- Interpret the information provided in manual pages.

---

### What is the `man` Command?

The `man` command, short for "manual," provides a comprehensive reference for most commands and utilities in Linux. Each command typically has an associated manual page (or "man page") that explains what the command does, how to use it, and what options are available.

---

### Using the `man` Command

To use the `man` command, simply type `man` followed by the name of the command you want to learn more about. For example, to read the manual page for the `ls` command:

```bash
man ls
```

This will open the manual page for `ls`, displaying detailed information on how to use it.

![Man command](/learn/linux_intro/assets/man.png){:class="w-100 img-fluid card-hover card-shadow rounded-3"}

---

### Navigating the Manual Pages

Once inside a manual page, you can navigate using the following keys:

- **Arrow Keys**: Move up and down line by line.
- **Spacebar**: Move down one screen at a time.
- **b**: Move up one screen at a time.
- **/search_term**: Search for a specific term within the page (press `n` to jump to the next occurrence).
- **q**: Quit the manual page and return to the command prompt.

---

### Understanding the Structure of a Man Page

Most man pages follow a similar structure, typically including the following sections:

1. **NAME**: The name of the command and a brief description.
1. **SYNOPSIS**: A summary of how to use the command, showing the command’s syntax, including options and arguments.
1. **DESCRIPTION**: A detailed explanation of what the command does.
1. **OPTIONS**: Lists all the options available with the command, often including examples.
1. **EXAMPLES**: Practical examples of how to use the command.
1. **SEE ALSO**: References to related commands or further reading.

For example, in the `ls` man page, the OPTIONS section would describe flags like `-l` (long listing format) and `-a` (include hidden files).

---

### Finding the Right Man Page

Sometimes, you might not know the exact command but want to search for a keyword. You can do this using:

```bash
man -k search_term
```

This command searches the short descriptions in all the man pages for the keyword you provide and returns a list of relevant commands. For example:

```bash
man -k directory
```

This might return commands like `mkdir`, `rmdir`, and `ls`, which all relate to directories.

---

## Summary

In this lesson, you learned how to use the `man` command to access detailed documentation for Linux commands. Understanding how to navigate and interpret man pages is crucial for mastering the command line and solving problems independently.

---

## Practice Exercise

Try using the `man` command to learn more about the following commands:

1. `man pwd`
1. `man mkdir`
1. `man chmod`

Explore the options and examples provided in each man page to deepen your understanding of these commands.

---
