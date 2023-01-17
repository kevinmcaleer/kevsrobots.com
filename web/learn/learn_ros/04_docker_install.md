---
layout: lesson
title: Docker Install
author: Kevin McAleer
type: page
previous: 03_install_updates.html
next: 05_get_cubie.html
description: Install and configure Docker
percent: 35
duration: 2
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
    - name: Talker ROS2 Program
      link: 10_listener.html
  - section: ROS2 Python Packages
    content:
    - name: Create a ROS2 Python Package
      link: 11_create_py_package.html
    - name: Create a ROS2 Python Publisher
      link: 12_create_publisher.html
    - name: Create a ROS2 Python Subscriber
      link: 13_create_subscriber.html
---


![Screenshot of the imager tool](assets/rpi_desktop.jpg){:class="cover"}

## Get Docker

We will use Docker to manage our ROS environment. Docker enables us to run containers; each container is a separate environment and can contain its own files, and run its own processes. Containers are like lightweight Virtual Machines, without the need to emulate an entire computer.

To get the Docker installation script:

* **Get the install script** - From the terminal, type:

```bash
curl -fsSL https://get.docker.com -o get-docker.sh
chmod +x get-docker.sh 
```

---

## Remove any existing docker installation

If you want to remove any existing docker installations, do this step before installing docker.

* **Purge** - From the terminal, type:

```bash
sudo apt-get purge docker-ce docker-ce-cli containerd.io -y
```

---

## Install Docker

* **Run the script** - From the terminal, type:

```bash
./get-docker.sh
```

---

## Fix permissions

Docker can run commands from a regular user account, but first we need to fix the permissions

* **Make pi user execute docker commands** - From the terminal, type:

```bash
sudo usermod -aG docker pi
```

*Where `Pi` is the user account you created when [setting up the Pi](02_pi_setup#setup-the-sd-card-using-raspberry-pi-imager)*

---

* **Unmask docker**- From the terminal, type:

```bash
sudo systemctl unmask docker
```

---

* **Fix permissions** - From the terminal, type:

```bash
sudo chmod 666 /var/run/docker.sock
```

---

* **install docker-compose** - From the terminal, type:

```bash
 pip3 -v install docker-compose
```

---

* **Start docker** - From the terminal, type:

```bash
sudo systemctl start docker
```

---

* **Reboot** - Restart the Pi to implement the changes. From the terminal, type:

```bash
sudo init 6
```

---
