---
layout: lesson
title: Talker ROS2 Program
type: page
description: Let's create a talker program to publish messages
---

## Talker Program

Let's create another program. This time we'll write our node as a class, which is how most ROS 2 nodes are written. Save it as `talker.py` next to your first program.

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

## What the code means

* `class talker(Node)` makes our own node by building on (inheriting from) ROS's `Node` class.
* `super().__init__("node_test")` gives the node its name.
* `self.create_timer(0.5, self.timer_callback)` asks ROS to call `timer_callback` every 0.5 seconds.
* Each time `timer_callback` runs, it adds one to the counter and logs a message, so you'll see `Hello1`, `Hello2`, `Hello3`... until you press `CTRL + C`.

Run it with `python3 src/talker.py`. In the next section we'll turn this into a real publisher that sends messages other nodes can receive.

## Check your understanding

{% quiz %}
? What is a ROS **node**?
- [x] A single program in a ROS system, usually doing one job
- [ ] A cable connecting two robots
- [ ] A type of sensor
> A robot is made of many nodes working together, for example one per sensor or motor.

? What does `rclpy.spin(node)` do?
- [ ] Spins the robot's wheels
- [x] Keeps the node running and handling events, like timers, until it's stopped
- [ ] Builds the workspace
{% endquiz %}
