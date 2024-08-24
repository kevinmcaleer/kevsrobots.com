---
title: Monitoring System Performance
description: Learn how to monitor system performance using commands like df, du, top, and free.
layout: lesson
cover: /learn/linux_intro/assets/performance.webp
---

![cover image]({{page.cover}}){:class="cover"}

## Introduction

Monitoring system performance is crucial for keeping your Raspberry Pi running smoothly. This lesson will introduce you to commands that help you keep track of disk usage, CPU load, and memory usage.

---

## Learning Objectives

- Check disk usage with `df` and `du`.
- Monitor CPU and process activity with `top` and `htop`.
- View memory usage with `free`.

---

### Checking Disk Usage

- **`df` (disk free)**: Displays the amount of disk space used and available on all mounted filesystems.

        df -h

    **Note** `-h`: Displays output in human-readable format (e.g., KB, MB, GB).

- **`du` (disk usage)**: Estimates file space usage. Use it to check how much space a directory is using.

        du -sh /home/pi

    **Note** `-s`: Displays only a total for each argument. `-h`: Displays output in human-readable format.

---

## Monitoring Processes with `top` and `htop`

- **`top`**: Displays real-time information about running processes, including CPU and memory usage.

        top

- **`htop`**: A more user-friendly version of top with an easier-to-read interface. You may need to install it first:

        sudo apt install htop
        htop

---

## Checking Memory Usage with `free`

The free command displays the total, used, and available memory on your Raspberry Pi. For human-readable output, use:

        free -h

---

## Summary

In this lesson, you learned how to monitor system performance using commands like `df`, `du`, `top`, and `free`. Regular monitoring can help you detect and troubleshoot potential issues before they become serious problems.

---
