---
layout: lesson
title: Create a ROS2 Python Package
type: page
description: Lets create a new Python Package
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
