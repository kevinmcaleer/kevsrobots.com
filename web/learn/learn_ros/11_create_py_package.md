---
layout: lesson
title: Create a ROS2 Python Package
author: Kevin McAleer
type: page
cover: /learn/learn_ros/assets/banners/11_create_py_package.jpg
date: 2023-01-07
previous: 10_listener.html
next: 12_create_publisher.html
description: Let's create a new Python Package
percent: 60
duration: 4
quiz_total: 58
snakie: false
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

By the end of this lesson you will be able to create a ROS 2 Python package with `ros2 pkg create`, build it with `colcon`, and add a dependency in `package.xml`.

## Why packages?

So far we ran scripts with `python3`. That doesn't scale: another node can't find your script, and you can't launch it with `ros2 run`. A **package** bundles nodes together with a description of what they need, so ROS can build, install and run them from anywhere.

## Create a ROS2 Package

* **Create the package** - from the container terminal (after sourcing `/opt/ros/humble/setup.bash`), type:

```bash
cd /ros2
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
cd my_py_pkg
```

`--build-type ament_python` says this is a Python package, and `--dependencies rclpy` records that it uses the `rclpy` library. ROS 2 prints what it created. Inside the new `my_py_pkg` folder you'll find:

```text
my_py_pkg/
  my_py_pkg/        <- your Python code goes here (has an __init__.py)
  resource/
  test/
  package.xml       <- name, version, maintainer, dependencies
  setup.cfg
  setup.py          <- how to install the package, and its entry points
```

> Normally packages live inside a workspace's `src` folder. To keep this course simple we're building the package folder directly, which works fine for learning.

`package.xml` holds the version, description, maintainer contact details and licence, plus the dependency on `rclpy` we asked for. ROS 2 fills in placeholders such as `TODO: Package description`; edit them if you plan to share the package.

---

## Build the new package

```bash
colcon build
```

You should see `Summary: 1 package finished`. New `build`, `install` and `log` folders appear in the package folder.

> ### "1 package had stderr output"
>
> On some setups you may see:
>
> ```text
> Summary: 1 package finished [3.60s]
>   1 package had stderr output: my_py_pkg
> ```
>
> That's a deprecation warning from newer versions of Python's `setuptools`, not a failure: the package still built. If you want it gone, install a known good version:
>
> ```bash
> sudo apt update && sudo apt install python3-pip -y
> pip install setuptools==58.2.0
> ```
>
> Then run `colcon build` again.
{:class="bg-blue"}

---

## Fix permissions

As before, the files belong to root, so let VS Code write to them:

```bash
chmod 777 -R /ros2/my_py_pkg
```

---

## Add a dependency to our package.xml

Our publisher and subscriber will use the `String` message from the `std_msgs` package. Tell ROS about that dependency:

* **Open `package.xml`** - in VS Code, inside the `my_py_pkg` folder
* **Add the dependency** - underneath `<depend>rclpy</depend>`, add:

```xml
<depend>std_msgs</depend>
```

**Why this matters:** dependencies let tools such as `rosdep` install what a package needs on another robot, and they make `colcon` build packages in the right order.

---

## Try it Yourself

1. Open `package.xml` and change the `description` to something meaningful, and put your name in `maintainer`.
2. Run `ros2 pkg list | grep my_py_pkg` after sourcing `install/setup.bash`. Does it appear?
3. Create a second package called `my_second_pkg`, with the same options. What differs?
4. Challenge: run `ros2 pkg executables my_py_pkg`. Why is the list empty at the moment?

## Common Issues

- **Problem**: `ros2: command not found`.
- **Solution**: Source `/opt/ros/humble/setup.bash` in this terminal.
- **Why**: Each new terminal starts without the ROS commands.

- **Problem**: `colcon build` says "no packages found" or builds nothing.
- **Solution**: Run it from the folder that contains `package.xml` (or the workspace `src` parent).
- **Why**: `colcon` looks for packages starting at the current folder.

- **Problem**: `package 'my_py_pkg' already exists`.
- **Solution**: Pick a new name or delete the old folder.
- **Why**: `ros2 pkg create` won't overwrite a package.

## Check your understanding

{% quiz %}
? Which file lists a package's dependencies?
- [ ] `setup.cfg`
- [x] `package.xml`
- [ ] `log`
> `package.xml` records the name, version, maintainer and `<depend>` entries.

? What does `--build-type ament_python` do?
- [x] Makes a Python package rather than a C++ one
- [ ] Builds the Docker image
- [ ] Installs Python
> ROS 2 supports both languages, and the build type picks which template to create.

? "1 package had stderr output" after `colcon build`. What does it mean?
- [ ] The build failed
- [x] A warning was printed, but the package still built
- [ ] The container crashed
> It's a `setuptools` deprecation warning, which you can fix by installing an older version.
{% endquiz %}
