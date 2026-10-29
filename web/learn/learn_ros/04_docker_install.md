---
layout: lesson
title: Docker Install
type: page
description: Install and configure Docker
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

Docker can run commands from a regular user account, but first we need to add our user to the `docker` group

* **Let your user run docker commands** - From the terminal, type:

```bash
sudo usermod -aG docker $USER
```

*`$USER` is filled in automatically with the user account you created when [setting up the Pi](02_pi_setup.html#setup-the-sd-card-using-raspberry-pi-imager)*

---

* **Unmask docker**- From the terminal, type:

```bash
sudo systemctl unmask docker
```

---

> ## Docker Compose
>
> The Docker install script also installs **Docker Compose**, which we'll use to start our ROS container. It's run as `docker compose` (with a space). Older guides use `docker-compose` with a hyphen, which was a separate program you no longer need.
{:class="bg-blue"}

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

## Check your understanding

{% quiz %}
? Why add your user to the `docker` group?
- [ ] To make Docker run faster
- [x] So you can run `docker` commands without typing `sudo` every time
- [ ] To install Docker Compose
> The change takes effect after you log out and back in, which is why we reboot.

? How is a Docker container different from a virtual machine?
- [x] Containers share the host's operating system kernel, so they're much lighter
- [ ] Containers emulate a whole computer, including the hardware
- [ ] There is no difference
{% endquiz %}
