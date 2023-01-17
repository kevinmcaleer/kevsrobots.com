---
layout: lesson
title: Create a ROS2 Python Publisher
author: Kevin McAleer
type: page
previous: 11_create_py_package.html
next: 13_create_subscriber.html
description: Lets create a new Python Publisher program
percent: 91
duration: 4
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


## Create a new Python file called pub.py

* **Create a new Python file** - from the docker terminal type:

```bash
touch my_py_pkg/pub.py
chmod -R 777 my_py_pkg
```

This will create the new empty file, and fix the permissions so we can write to it from VS Code.

---

## Edit pub.py

Cut and paste the following code into the new file:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):

    def __init__(self):
        super().__init__("node_test")
        self.counter_ = 0
        # create_publisher needs 3 parameters: Msg Type, message, and buffer size
        self.publisher = self.create_publisher(String, "Hello", 10)
        self._timer = self.create_timer(0.5, self.publish_hello)
        self.get_logger().info("publishing message")

    def publish_hello(self):
        msg = String()
        msg.data = "Hello World"
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Talker()
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
        "talker = my_py_pkg.talker:main"
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

* **Build with colcon** - From the docker terminal type:

```bash
colcon build
```

---

## Testing the publisher works

Lets test out our code so far...

* **Source install/setup.bash** - From the docker terminal type:

``` bash
source install/setup.bash
```

* **Run the package** - From the docker terminal type:

``` bash
ros2 run my_py_pkg pub 
```

The package should now run and you iwll see the logger message `[INFO] [1673801367.468008815] [node_test]: publishing message`.

You can also see the messages themselves by querying the ROS2 topic, via the commandline:

* **Echo the topic** - From the docker terminal type:

``` bash
ros2 topic echo Hello
```

This will show the output from the ROS2 topic `Hello`, which is the topic we are publishing too.

---
