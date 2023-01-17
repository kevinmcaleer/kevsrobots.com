---
layout: lesson
title: Create a ROS2 Python Subscriber
author: Kevin McAleer
type: page
previous: 12_create_publisher.html
description: Lets create a new Python Subscriber program
percent: 100
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
---


## Create a new Python file called sub.py

* **Create a new Python file** - from the docker terminal type:

```bash
touch my_py_pkg/sub.py
chmod -R 777 my_py_pkg
```

This will create the new empty file, and fix the permissions so we can write to it from VS Code.

---

## Edit sub.py

Cut and paste the following code into the new file:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):

    def __init__(self):
        super().__init__("node_test")
        self.counter_ = 0
        # create_subscriber needs 3 parameters: Msg Type, topic name, the callback, and queue size buffer
        self.subscriber = self.create_subscription(String, "Hello", self.callback_hello, 10)
        self.get_logger().info("Subscribed to Hello topic")

    def callback_hello(self, message):
        self.get_logger().info(message.data)
     
def main(args=None):
    rclpy.init(args=args)
    node = Listener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

---

## Add the entry_point to setup.py

We now need to update `setup.py` to include the entry_point for our code. An `entry point` is the name of the python scripts that are available to run from with the package, and are shown in the list when you type `ros2 run my_py_pkg`.

* **Update setup.py** - From the VS Code open setup.py from with `my_py_pkg` folder
* **Add the entry points** - add the following lines to the entry point list:

```python
entry_points={
    'console_scripts' : [
        "pub = my_py_pkg.pub:main",
        "talker = my_py_pkg.talker:main",
        "listener = my_py_pkg.sub:main"
    ],
},
```

---

## Build the package

We can now build out package using `colcon build`. Do this from the root of the workspace (in this case `/ros2/my_py_pkg`):

* **change to the root of the workspace**- From the docker terminal type:

```bash
cd /ros2/my_py_pkg
```

* **Build with colcon and source the new setup.bash** - From the docker terminal type:

```bash
colcon build
source install/setup.bash
```

---
