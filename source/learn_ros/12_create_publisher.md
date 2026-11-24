---
layout: lesson
title: Create a ROS2 Python Publisher
type: page
description: Let's create a new Python Publisher program
cover: /learn/learn_ros/assets/banners/12_create_publisher.jpg
snakie: false
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to write a publisher node, register it as an entry point in `setup.py`, and watch its messages with `ros2 topic echo`.

## How publishing works

A **publisher** sends messages to a named **topic**. Anything interested can subscribe to that topic, and the publisher doesn't need to know who's listening. Our publisher sends a `String` message to a topic called `Hello` twice a second.

```mermaid
flowchart LR
    P[hello_publisher node] -- std_msgs/String --> T((Hello topic))
    T --> E[ros2 topic echo]
```

## Create a new Python file called pub.py

* **Create the file** - from the container terminal, inside the `/ros2/my_py_pkg` folder, type:

```bash
touch my_py_pkg/pub.py
chmod -R 777 my_py_pkg
```

This creates the empty file inside the *inner* `my_py_pkg` folder (where the package's Python code lives), and fixes the permissions so VS Code can write to it.

## Edit pub.py

Cut and paste the following code into the new file:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):

    def __init__(self):
        super().__init__("hello_publisher")
        # create_publisher needs 3 parameters: message type, topic name, and queue size
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

The new parts are `from std_msgs.msg import String` (the message type, which is why we added `std_msgs` to `package.xml`) and `create_publisher(String, "Hello", 10)`. The `10` is the queue size: how many messages to hold if a subscriber falls behind.

---

## Add the entry_point to setup.py

An **entry point** is a name that `ros2 run` can launch. It maps `pub` to the `main` function in `my_py_pkg/pub.py`.

* **Open `setup.py`** - in VS Code, from the `my_py_pkg` folder
* **Fill in the entry points** - find the empty `console_scripts` list and change it so it reads:

```python
    entry_points={
        'console_scripts': [
            "pub = my_py_pkg.pub:main",
            "counter = my_py_pkg.counter:main",
        ],
    },
```

*The `counter` entry expects a `counter.py` in the same folder as `pub.py`: copy the counter node from [the previous lesson](10_listener.html) into `my_py_pkg/counter.py` so `ros2 run my_py_pkg counter` works too.*

---

## Build the package

Do this from the package folder:

```bash
cd /ros2/my_py_pkg
colcon build
```

## Testing the publisher works

* **Source the package** - from the container terminal type:

```bash
source install/setup.bash
```

* **Run the package** - type:

```bash
ros2 run my_py_pkg pub
```

You'll see the logger message `[INFO] [1673801367.468008815] [hello_publisher]: publishing message`. The time number will differ. The node then runs quietly, publishing.

In a **second** container terminal (source both setup files again), look at what's happening:

```bash
ros2 topic list
ros2 topic echo Hello
```

`ros2 topic list` shows `/Hello` alongside `/parameter_events` and `/rosout`. `ros2 topic echo` prints each message:

```text
data: Hello World
---
data: Hello World
---
```

You can also run `ros2 topic info /Hello` to see the message type `std_msgs/msg/String` and that there's one publisher, or `ros2 node list` to see `/hello_publisher`.

---

## Try it Yourself

1. Change the message text from `Hello World` to something else, rebuild and run it. Does the echo change?
2. Change the timer so it publishes once a second. Check it with `ros2 topic hz Hello`.
3. Add a counter, so each message says `Hello World 1`, `Hello World 2`, and so on.
4. Challenge: publish a different topic name, `Greetings`, and echo it. What happens to `Hello`?

## Common Issues

- **Problem**: `No executable found` when running `ros2 run my_py_pkg pub`.
- **Solution**: Check the entry point in `setup.py`, then rebuild with `colcon build` and `source install/setup.bash` again.
- **Why**: `ros2 run` only knows about entry points that were present at the last build.

- **Problem**: `ModuleNotFoundError: No module named 'std_msgs'`.
- **Solution**: Source `/opt/ros/humble/setup.bash` first.
- **Why**: `std_msgs` is a ROS package, found through the setup script.

- **Problem**: `ros2 topic echo` prints nothing.
- **Solution**: Check the publisher is running, and that the topic name is spelled the same, including capitals.
- **Why**: Topic names are case sensitive: `Hello` and `hello` are different topics.

## Check your understanding

{% quiz %}
? What are the three arguments to `create_publisher(String, "Hello", 10)`?
- [x] The message type, the topic name and the queue size
- [ ] The node name, the timer and the message
- [ ] The package, the file and the function
> The queue size is how many messages are held if a subscriber falls behind.

? What does an entry in `console_scripts` do?
- [ ] Installs Python
- [x] Gives `ros2 run` a name for a function it can start
- [ ] Creates a topic
> `pub = my_py_pkg.pub:main` means `ros2 run my_py_pkg pub` calls `main` in `pub.py`.

? Which command lists the topics currently in use?
- [ ] `ros2 pkg list`
- [x] `ros2 topic list`
- [ ] `colcon list`
{% endquiz %}
