---
title: Managing Software
description: Learn how to remove, update, and manage installed software using apt.
layout: lesson
cover: /learn/linux_intro/assets/uninstall.jpg
---

![cover image]({{page.cover}}){:class="cover"}

## Introduction

Managing installed software is just as important as installing it. This lesson will teach you how to remove, update, and manage software packages using the `apt` package manager.

---

## Learning Objectives

- Remove software packages using `apt remove` and `apt purge`.
- Update installed software using `apt upgrade`.
- Clean up unnecessary packages with `apt autoremove`.

---

### Removing Software

If you no longer need a software package, you can remove it using `apt remove`. For example:

        sudo apt remove git

To remove a package and its configuration files, use apt purge:

        sudo apt purge git

---

### Updating Installed Software

To update all installed packages to their latest versions, use:

        sudo apt upgrade

For a more thorough upgrade that handles dependencies better, use:

        sudo apt full-upgrade

## Cleaning Up Unnecessary Packages

Over time, unused packages can accumulate on your system. To remove packages that are no longer needed, use:

        sudo apt autoremove

---

## Summary

In this lesson, you learned how to manage installed software by removing, updating, and cleaning up packages using `apt`. Keeping your system clean and up-to-date is essential for maintaining performance and security.

---
