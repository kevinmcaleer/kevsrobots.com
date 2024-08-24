---
title: Networking Commands
description: Learn how to check network status and test connectivity using commands like ifconfig, ip, ping, and traceroute.
layout: lesson
cover: /learn/linux_intro/assets/ipconfig.jpg
---

![cover image]({{page.cover}}){:class="cover"}

## Introduction

Networking is a key aspect of using your Raspberry Pi, especially for projects that involve remote access or internet connectivity. In this lesson, you'll learn how to check your network status and test connectivity using essential networking commands.

---

## Learning Objectives

- Check network status using `ifconfig` and `ip`.
- Test network connectivity using `ping` and `traceroute`.
- Connect to another machine using `ssh`.

---

### Checking Network Status

- **`ifconfig`**: Displays network configuration details such as IP addresses and MAC addresses.

      ifconfig

- **`ip`**: A more modern and powerful tool for network management. To view your network interfaces, use:

      ip a

---

## Testing Network Connectivity

- **`ping`**: Tests the connection to another device on the network by sending ICMP echo requests.

      ping google.com

- **`traceroute`**: Displays the route that packets take to reach a destination.

      traceroute google.com

---

## Using SSH to Connect Remotely

The `ssh` (**Secure Shell**) command allows you to securely connect to another machine over a network. To connect to a Raspberry Pi with the IP address 192.168.1.10, use:

    ssh pi@192.168.1.10

You'll be prompted to enter the password for the `pi` user.

---

## Summary

In this lesson, you learned how to check network status, test connectivity, and connect to another machine using SSH. These commands are essential for managing and troubleshooting network-related tasks on your Raspberry Pi.

---
