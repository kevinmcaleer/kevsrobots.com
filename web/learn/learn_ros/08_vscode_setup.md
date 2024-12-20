---
layout: lesson
title: VSCode setup
author: Kevin McAleer
type: page
cover: /learn/learn_ros/assets/learn_ros.jpg
date: 2023-01-07
previous: 07_build_container.html
next: 09_first_ros_program.html
description: Install the VSCode extensions so we can remotely connect to the Raspberry
  Pi and Docker container instances
percent: 45
duration: 3
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
---


## VSCode Remote connection

Microsoft VSCode can connect to a remote computer, such as our Raspberry Pi, over SSH and then connect to running contains on that remote machine so that we can write, run and debug our ROS2 programs.

We will need to install some extensions first, then connect to our ROS2 container on the Raspberry Pi.

---

## Install Extensions

Lets install the two docker extensions on our main computer (not the Raspberry Pi), that we will use to write, run and debug code.

![VS Code Extentions](assets/vscode_extensions.png){:class="img-fluid w-50 shadow-lg"}

* **Click on the Extensions Icon** - This will list installed and installable extensions
* **Type `docker` in the search bar** - This will show installable docker extensions
* **Click Install on the `Docker` extension** - This extension is published by Microsoft
* **Type `remote` in the search bar** - This will show installable remote extensions
* **Click Install on the `Remote - SSH` extension** - This extension is published by Microsoft
* **Click Install on the `Remote Explorer` extension** - This extension is also published by Microsoft

---

## Remote connect

We can now connect to the Raspberry Pi 4 running our new docker container by using SSH from our main computer running VSCode.

* **Open Command Palette** - Hold `CTRL + SHIFT + P` to open the command palette
* **Connect to Host** - Type `connect` in the command palette search and then select Connect
* **Click `Remote-SSH Connect to Host...`** - Then type `pi@192.168.1.4` where `pi` is the username you selected [earlier](02_pi_setup#setup-the-sd-card-using-raspberry-pi-imager) and the `IP Address` is that of your Pi.

![Screenshot of remote-ssh](assets/remote_ssh01.png){:class="img-fluid w-100 shadow-lg"}

> *To find the ip address of your Pi you can type `ip a` from the Raspberry Pi terminal; there will be a list
> of all the addresses the Pi is using.*

* **Type your Pi's password** - Type the password you created in [Step 2](02_pi_setup#setup-the-sd-card-using-raspberry-pi-imager)

![Screenshot of remote-ssh](assets/remote_ssh02.png){:class="img-fluid w-100 shadow-lg"}

* **You are now connected to the Pi remotely** - notice the green connection status at the bottom left of VSCode

![Screenshot of remote-ssh](assets/remote_ssh03.png){:class="img-fluid w-100 shadow-lg"}

* **Click on `Open Folder`** -  We can now open a folder; select 'Cubie-1'

![Screenshot of remote-ssh](assets/remote_ssh04.png){:class="img-fluid w-100 shadow-lg"}

*You can browse the remote file system and open the files we downloaded [earlier](05_get_cubie#get-cubie-1-files)*

![Screenshot of remote-ssh](assets/remote_ssh05.jpg){:class="img-fluid w-100 shadow-lg"}

---

## Connect to the container

We can now connect to the running docker container that we created in [step 7](07_build_container#run-the-container). We can create code from the comfort of our main computer running vscode and run code remotely on our robot. This will make the development process much simpler.

To connect to the container:

* **Open a terminal** - with <code>CTLR + SHIFT + `</code>
* **Connect to the running container** - From the terminal, type:

```bash
docker exec -it docker_ros2_1 bash
```

*Where `docker_ros2_1` is the name of the running container*

![Screenshot of remote-ssh](assets/remote_ssh07.png){:class="img-fluid w-100 shadow-lg"}

> To list all the running containers, type `docker ps` from the terminal

> 🎉 Congratulations, you've now setup ROS2 in a container on the Raspberry Pi and connected to it from another computer running VSCode.
{:class="bg-blue"}

---
