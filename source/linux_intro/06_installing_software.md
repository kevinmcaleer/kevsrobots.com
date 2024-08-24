---
title: Installing Software
description: Learn how to install software packages using the apt package manager.
layout: lesson
cover: /learn/linux_intro/assets/books.jpg
---

![cover image]({{page.cover}}){:class="cover"}

## Introduction

The `apt` package manager makes it easy to install, update, and manage software on Raspberry Pi OS. This lesson will teach you how to install software packages using `apt`.

---

## Learning Objectives

- Understand the purpose of a package manager.
- Use `apt` to update your package list and install software.
- Search for packages and resolve dependencies.

---

## What is a Package Manager?

A package manager like `apt` simplifies the process of installing and managing software.

---

### Understanding the APT Package Manager

The **Advanced Package Tool** (`APT`) is a powerful package management system used by Debian-based Linux distributions, including Ubuntu and Raspberry Pi OS.

It simplifies the process of managing software on your system by automating tasks like installation, upgrading, and removal of software packages.

---

## Updating the Package List

Before installing software, it's a good idea to update the package list. This ensures you have the latest information about available software packages. The Package List is a database of all available software packages.

        sudo apt update

This command fetches the latest package information from the repositories. It's a good practice to run this command before installing new software.

### About Repositories

Repositories are servers that store software packages. When you install software using `apt`, it downloads the package from a repository. The Raspberry Pi OS comes with a set of default repositories that contain a wide range of software packages.

You can also add additional repositories to access more software.

---

## Installing Software

To install a package, use the `apt install` command. For example, to install the git version control system, type:

        sudo apt install git

`apt` will automatically download and install the package along with any dependencies.

---

## Searching for Packages

If you're unsure of the exact package name, you can search the repositories using:

        apt search <package-name>

---

## Updating Installed Software

To update all installed packages to the latest versions, use:

        sudo apt upgrade

This command will download and install the latest versions of all installed packages. It will ask you if you want to proceed before making any changes; you can confirm by typing `Y` and pressing `Enter`. To automate this process, you can use the `-y` flag:

        sudo apt upgrade -y

---

## Summary

In this lesson, you learned how to install software using the `apt` package manager. This is a crucial skill for setting up and managing software on your Raspberry Pi.

---
