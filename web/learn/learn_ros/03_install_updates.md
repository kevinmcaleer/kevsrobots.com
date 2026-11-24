---
layout: lesson
title: Installing updates
author: Kevin McAleer
type: page
cover: /learn/learn_ros/assets/banners/03_install_updates.jpg
date: 2023-01-07
previous: 02_pi_setup.html
next: 04_docker_install.html
description: Update and Upgrade the OS to the latest patch level
percent: 20
duration: 3
quiz_total: 58
date_updated: 2026-11-24
navigation:
- name: Learn ROS with me
- content:
  - section: Overview
    content:
    - name: Introduction
      link: 00_intro.html
    - name: What is ROS
      link: 01_what_is_ros.html
  - section: Setting up the Raspberry Pi 4 environment
    content:
    - name: Raspberry Pi Setup
      link: 02_pi_setup.html
    - name: Installing updates
      link: 03_install_updates.html
  - section: Setting Docker for ROS
    content:
    - name: Docker Install
      link: 04_docker_install.html
    - name: Get Cubie-1 files
      link: 05_get_cubie.html
    - name: Clone Docker Images
      link: 06_clone_images.html
  - section: Build the ROS2 Container
    content:
    - name: Build the ROS2 Container
      link: 07_build_container.html
  - section: Setting up VSCode
    content:
    - name: VSCode setup
      link: 08_vscode_setup.html
  - section: First ROS2 Program
    content:
    - name: First ROS2 Program
      link: 09_first_ros_program.html
    - name: Nodes as Classes
      link: 10_listener.html
  - section: ROS2 Python Packages
    content:
    - name: Create a ROS2 Python Package
      link: 11_create_py_package.html
    - name: Create a ROS2 Python Publisher
      link: 12_create_publisher.html
    - name: Create a ROS2 Python Subscriber
      link: 13_create_subscriber.html
  - section: Docker and X11
    content:
    - name: Getting Rviz2 and RQT to work in docker
      link: 14_docker_x11.html
  - section: RViz2
    content:
    - name: Rviz2
      link: 15_rviz.html
    - name: Rviz2 Displays
      link: 16_rviz_displays.html
    - name: Transformations with TF2
      link: 17_tf2.html
    - name: Laserscan data
      link: 18_laserscan.html
    - name: Summary and next steps
      link: 19_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to bring a Raspberry Pi fully up to date from the terminal or the desktop update icon.

## Why update first?

A fresh image can be weeks or months old. Updating before you install Docker means you get the latest security fixes and a kernel that matches the packages Docker will want. It also catches network problems early: if `apt update` can't reach the internet, Docker's installer won't either, and it's better to find that out now.

## Update from the terminal

* **Connect to the Pi** - once it has booted, either attach a keyboard, mouse and monitor, or connect remotely with `ssh`
* **Open a terminal** and type:

```bash
sudo apt update
sudo apt upgrade -y
```

`apt update` refreshes the list of available packages. `apt upgrade` installs the newer versions. The `-y` answers "yes" for you. On a new image this can take several minutes.

If the upgrade installed a new kernel, restart the Pi:

```bash
sudo reboot
```

## Or use the desktop

If you are using the desktop, click the **update icon** next to the clock. It may not appear immediately after startup. Click it, then choose to install the updates. Both ways do the same job.

```mermaid
flowchart LR
    A[apt update<br/>refresh the list] --> B[apt upgrade<br/>install newer versions]
    B --> C{New kernel?}
    C -- yes --> D[sudo reboot]
    C -- no --> E[Ready for Docker]
    D --> E
```

---

## Try it Yourself

1. Run `sudo apt update` twice. What is different about the second run?
2. Run `apt list --upgradable` before upgrading to see what is waiting.
3. Challenge: use `cat /etc/os-release` to find the Debian version your Pi is running.

## Common Issues

- **Problem**: `apt update` shows "Temporary failure resolving".
- **Solution**: Check the Pi is on the network (`ip a`, or try `ping -c 3 raspberrypi.com`).
- **Why**: `apt` downloads from the internet, so no network means no packages.

- **Problem**: "Could not get lock /var/lib/dpkg/lock".
- **Solution**: Wait a minute, or close the desktop update window, then try again.
- **Why**: Only one program may use the package system at a time, and the updater may already be running.

## Check your understanding

{% quiz %}
? What does `sudo apt update` do?
- [ ] Installs the newest versions of everything
- [x] Refreshes the list of available packages
- [ ] Restarts the Pi
> The installing is done by `apt upgrade`.

? What does the `-y` in `sudo apt upgrade -y` do?
- [x] Answers "yes" to the confirmation question for you
- [ ] Upgrades only one package
- [ ] Skips the download

? When should you run `sudo reboot` after upgrading?
- [ ] Never
- [x] If the upgrade installed a new kernel
- [ ] Before `apt update`
> The diagram in the lesson shows the decision: new kernel means reboot, then carry on.
{% endquiz %}
