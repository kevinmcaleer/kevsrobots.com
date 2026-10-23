---
layout: lesson
title: Create a ROS2 Python Package
type: page
description: Let's create a new Python Package
---

## Create a ROS2 Package

A **package** bundles nodes together so they can be built, shared and run with `ros2 run`. Use the ROS2 `pkg create` command to create one.

* **Create the package** - From the ROS2 container terminal, type:

```bash
cd /ros2
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
cd my_py_pkg
```

This will create a new folder that includes the dependencies needed for this package (the `rclpy` library), as well as the files required to install this such as the `setup.py` file.

The `package.xml` file is also created. This contains the version, description, maintainer contact details as well as the licence type.
It also contains the dependencies for this package; the `rclpy` library we specified.

---

## Build the new package

Let's build the new package

```bash
colcon build
```

> ### 1 package had stderr output:
>
> If you get an error message that says:
>
> ```bash
> Summary: 1 package finished [3.60s]
>   1 package had stderr output: my_py_pkg
> ```
>
> This is a warning from newer versions of Python's `setuptools`. We can fix it by installing a known good version:
>
> * **Install pip3** - from the docker terminal type:
>
> `sudo apt update && sudo apt install pip -y`
>
> * **Downgrade setuptools** -  from the docker terminal type:
>
> `pip install setuptools==58.2.0`
>
> This should fix the error; type `colcon build` again to successfully build the package.

---

## Fix permissions

We need to fix the permissions to edit files in these folders, from within VSCode:

* **Change permissions on the folder** - From the docker commandline type:

```bash
chmod 777 -R /ros2/my_py_pkg
```

---

## Add a dependency to our package.xml

* **Open the package.xml** - In VSCode open the package.xml file within the my_py_pkg folder
* **Add the dependency** - add the following text underneath `<depend>rclpy</depend>`:

```xml
<depend>std_msgs</depend>
```

---
