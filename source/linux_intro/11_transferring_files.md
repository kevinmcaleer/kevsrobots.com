---
title: Transferring Files Over a Network
description: Learn how to securely transfer files between machines using scp and rsync.
layout: lesson
cover: /learn/linux_intro/assets/filecopy.jpg
---

![cover image]({{page.cover}}){:class="cover"}

## Introduction

Transferring files over a network is a common task, especially when working with multiple devices. In this lesson, you'll learn how to securely transfer files between machines using `scp` and `rsync`.

---

## Learning Objectives

- Transfer files securely between machines using `scp`.
- Use `rsync` for efficient file transfers and synchronization.

---

### Transferring Files with `scp`

The `scp` (secure copy) command is used to transfer files between two machines over a secure connection. For example, to copy a file from your Raspberry Pi to another machine:

    scp example.txt pi@192.168.1.10:/home/pi/

To copy a file from another machine to your Raspberry Pi:

    scp pi@192.168.1.10:/home/pi/example.txt /home/pi/

## Synchronizing Files with rsync

The `rsync` command is a powerful tool for efficiently transferring and synchronizing files between machines. It only transfers the differences between the source and the destination, making it faster than scp for large directories.

    rsync -avh /home/pi/Projects/ pi@192.168.1.10:/home/pi/Projects/

The `-avh` flags stand for archive mode, verbose output, and human-readable output, respectively.

---

## Summary

In this lesson, you learned how to securely transfer files between machines using `scp` and `rsync`. These tools are essential for managing files across multiple devices, especially in networked environments.

---
