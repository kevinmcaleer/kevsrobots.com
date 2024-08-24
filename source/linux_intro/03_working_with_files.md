---
title: Working with Files
description: Learn how to create, view, edit, and manage files using the command line.
layout: lesson
cover: /learn/linux_intro/assets/files.png
---

![cover image]({{page.cover}}){:class="cover"}

## Introduction

Files are at the heart of everything you do on your Raspberry Pi. This lesson will teach you how to create, view, edit, and manage files using various command-line tools.

---

## Learning Objectives

- Create files using the `touch` command.
- View file contents with commands like `cat`, `less`, `head`, and `tail`.
- Edit files using the `nano` text editor.

---

### Creating Files with `touch`

The `touch` command is used to create an empty file. For example, to create a file named `example.txt`, you would use:

```bash
touch example.txt
```

A new file called `example.txt` will be created in the current directory, you can confirm this by using the `ls` command and checking for the file.

![touch command](/learn/linux_intro/assets/touch.png){:class="w-100 img-fluid card-hover card-shadow rounded-3"}

---

## Viewing File Contents

- `cat`: Concatenates and displays the contents of a file.

        cat example.txt

- `less`: Views file content page by page, useful for large files.

        less example.txt

- `head`: Displays the first few lines of a file.

        head example.txt

- `tail`: Displays the last few lines of a file.

        tail example.txt

---

## Editing Files with nano

The nano editor is a simple, easy-to-use text editor. To edit example.txt, type:

```bash
nano example.txt
```

Within `nano`, you can type text, and use keyboard shortcuts to save and exit (e.g., `Ctrl + O` to save and `Ctrl + X` to exit).

> We'll cover `nano` in more detail in a later lesson.

---

## Summary

In this lesson, you learned how to create, view, and edit files using the command line. Mastering these commands is crucial for managing files on your Raspberry Pi efficiently.

---
