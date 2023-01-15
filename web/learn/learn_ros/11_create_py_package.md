---
layout: lesson
title: Create a ROS2 Python Package
author: Kevin McAleer
type: page
previous: 10_talker.html
next: 12_create_publisher.html
description: Lets create a new Python Package
percent: 84
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
      link: 10_talker.html
  - section: ROS2 Python Packages
    content:
    - name: Create a ROS2 Python Package
      link: 11_create_py_package.html
    - name: Create a ROS2 Python Publisher
      link: 12_create_publisher.html
    - name: Create a ROS2 Python Subscriber
      link: 13_create_subscriber.html
---


## Create a ROS2 Package

Use the ROS2 `pkg create` command to create .

```bash
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
```

This will create a new folder that includes the dependencies needed for this package (the `rclpy` library), as well as the files required to install this such as the `setup.py` file.

The `package.xml` file is also created. This contains the version, description, maintainer contact details as well as the licence type.
It also contains the depednecnies for this package; the `rcply` library we specified.

---

## Build the new package

Lets build the new package

```bash
colcon build
```

> ### 1 package had sdrerr output:
>
> If you get and error message that says:
>
> ```bash
> Summary: 1 package finished [3.60s]
>   1 package had stderr output: my_py_pkg
> ```
>
> This was because there is a bug in the setuptooldependency package, we can fix this by using a known good version of pip3:
>
> * **Install pip3** - from the docker terminal type:
>
> `sudo apt update && sudo apt install pip -y`
>
> * **Downgrade pip** -  from the docker terminal type:
>
> `pip install setuptools==58.2.0`
>
> This should fix the error; type `colcon build` again to successfully build the package.

---

## Fix permissions

We need to fix the permissions to edit files in these folders, from with VSCode:

* **Change permisions on the folder** - From the docker commandline type:

```bash
chmod 777 -R  my_py_pkg/
```

---

## Add a dependency to our package.xml

* **Open the package.xml** - In VSCode open the package.xml file within the my_py_pkg folder
* **Add the depenecy** - add the following text underneath `<depend>rclpy</depend>`:

```xml
<depend>std_msgs</depend>
```

---
