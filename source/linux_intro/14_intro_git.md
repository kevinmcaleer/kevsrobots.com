---
title: Introduction to Git
description: Learn the basics of Git, a version control system, and how to set it up on Raspberry Pi OS.
layout: lesson
cover: /learn/linux_intro/assets/git.jpg
---

![cover image]({{page.cover}}){:class="cover"}

## Introduction

`git` is a powerful version control system that helps you manage changes to your code over time. This lesson will introduce you to Git and show you how to set it up on your Raspberry Pi.

---

## Learning Objectives

- Understand what Git is and why it's useful.
- Install Git on Raspberry Pi OS.
- Configure Git with your user information.

---

### What is Git?

Git is a distributed version control system that tracks changes to files and allows multiple people to collaborate on a project. It’s widely used in software development but is also useful for managing any kind of file.

---

### Installing Git on Raspberry Pi OS

To install Git on your Raspberry Pi, use the `apt` package manager:

        sudo apt update
        sudo apt install git

---

## Configuring Git

After installing Git, you need to configure it with your name and email address. These details will be used in your commits:

        git config --global user.name "Your Name"
        git config --global user.email "your.email@example.com"

---

## Summary

In this lesson, you learned what Git is, how to install it on Raspberry Pi OS, and how to configure it with your user information. Git is an essential tool for managing your projects, especially as they grow in complexity.

---
