---
layout: lesson
title: Create a ROS2 Python Subscriber
type: page
description: Lets create a new Python Subscriber program
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
