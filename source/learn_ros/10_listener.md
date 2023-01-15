---
layout: lesson
title: Listener ROS2 Program
type: page
description: Lets create a listener program to show the talker messages
---

## Listener Program

Lets create another program, that will print out every message that the talker program produces.

This time we'll make it in a class:

```python

import rclpy
from rclpy.node import Node

class talker(Node):

    def __init__(self):
        super().__init__("node_test")
        self.counter_ = 0
        self.get_logger().info("Hello World")
        self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.counter_ += 1
        self.get_logger().info("Hello" + str(self.counter_))

def main(args=None):
    rclpy.init(args=args)
    node = talker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

```

---
