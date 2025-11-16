---
title: "Arduino Alvik Maze Navigation - From Wall Following to SLAM"
description: >-
    Learn how to navigate an Arduino Alvik robot through a maze, starting with simple wall following and advancing to ROS2 SLAM
excerpt: >-
    Take your Arduino Alvik from basic wall following to advanced SLAM-based maze navigation with ROS2
layout: showcase
date: 2025-11-16
author: Kevin McAleer
difficulty: intermediate
cover: /assets/img/blog/alvik_maze/cover.jpg
hero: /assets/img/blog/alvik_maze/hero.png
mode: light
tags:
 - Arduino
 - Alvik
 - MicroPython
 - Robotics
 - ROS2
 - SLAM
 - Navigation
groups:
 - robots
 - micropython
 - arduino
videos:
 - vHUXDgSiWio
code:
 - https://www.github.com/kevinmcaleer/alvik_maze
stl:
  - name: Maze Stand
    link: /assets/stl/alvik_maze/maze_stand.stl
    description: A 3D printable stand system to create modular maze walls

---

## Navigating a Maze with Arduino Alvik

Ahoy there makers! Today we're tackling one of robotics' classic challenges: maze navigation. We'll start with a simple wall-following algorithm and work our way up to a sophisticated ROS2-enabled SLAM (Simultaneous Localization and Mapping) solution using the Arduino Alvik robot.

This is the perfect progression from beginner to advanced autonomous navigation!

---

## What You'll Learn

By the end of this guide, you'll understand:

- **Wall Following Algorithm** - The classic left-hand rule
- **Sensor Integration** - Using Alvik's time-of-flight sensors
- **Navigation Logic** - Making decisions based on sensor data
- **ROS2 Integration** - Connecting Alvik to the Robot Operating System
- **SLAM Technology** - Building maps while navigating
- **Autonomous Navigation** - Path planning through known spaces

---

## Hardware Requirements

- **Arduino Alvik Robot** ([available from Arduino](https://store.arduino.cc/products/alvik))
- **Maze or walls** - Cardboard boxes work great!
- **Computer with ROS2** (optional, for advanced SLAM section)
- **WiFi connection** (for ROS2 communication)

### 3D Printed Maze Stand

Want to build a proper maze? I've designed a 3D printable maze stand system that makes it easy to create modular maze walls!

**Download the STL file here:** [Maze Stand STL](/assets/stl/alvik_maze/maze_stand.stl)

The maze stand features:
- Modular design for flexible maze layouts
- Easy assembly and reconfiguration
- Stable base for reliable wall detection
- Compatible with cardboard or foam board walls

Simply print multiple stands and slot in your wall materials to create custom maze configurations!

---

## Part 1: Simple Wall Following

Let's start with the classic wall-following algorithm. This simple but effective approach will get your Alvik through many mazes!

### The Left-Hand Rule

The algorithm is beautifully simple:
1. Keep your left hand on the wall
2. Follow the wall around
3. Eventually, you'll find the exit!

This works for any simply-connected maze (one without loops or islands).

### Basic Wall Following Code

Here's a simple implementation using Alvik's time-of-flight sensors:

```python
from arduino_alvik import ArduinoAlvik
import time

# Initialize Alvik
alvik = ArduinoAlvik()
alvik.begin()

# Distance thresholds (in cm)
WALL_DISTANCE = 15  # Desired distance from wall
TOO_CLOSE = 10      # Too close to wall
TOO_FAR = 20        # Too far from wall
FRONT_OBSTACLE = 20 # Obstacle ahead

def wall_follow_left():
    """
    Follow the left wall using simple rules
    """
    while True:
        # Read distance sensors
        left = alvik.get_distance_left()
        front = alvik.get_distance_center()
        right = alvik.get_distance_right()

        # Decision logic
        if front < FRONT_OBSTACLE:
            # Wall ahead - turn right
            alvik.rotate(90)

        elif left > TOO_FAR:
            # No wall on left - turn left to find it
            alvik.rotate(-90)
            alvik.drive(10, 0)  # Move forward a bit

        elif left < TOO_CLOSE:
            # Too close to left wall - drift right
            alvik.drive(10, 30)

        else:
            # Perfect distance - go straight
            alvik.drive(10, 0)

        time.sleep(0.1)

# Run the wall following algorithm
try:
    wall_follow_left()
except KeyboardInterrupt:
    alvik.stop()
    print("Stopped by user")
```

### How It Works

1. **Sensor Reading** - Continuously monitor left, front, and right distances
2. **Decision Making** - Use simple if/else logic to decide what to do
3. **Motor Control** - Adjust speed and direction based on decisions

The beauty of this algorithm is its simplicity - no mapping, no complex planning, just reactive behavior!

---

## Part 2: Enhanced Wall Following

Let's improve our basic algorithm with smoother control and better obstacle handling:

```python
def enhanced_wall_follow():
    """
    Improved wall following with proportional control
    """
    BASE_SPEED = 15
    KP = 2.0  # Proportional gain

    while True:
        left = alvik.get_distance_left()
        front = alvik.get_distance_center()

        # Check for front obstacle
        if front < FRONT_OBSTACLE:
            alvik.stop()
            alvik.rotate(90)
            continue

        # Proportional control for smooth following
        error = WALL_DISTANCE - left
        turn_rate = error * KP

        # Constrain turn rate
        turn_rate = max(-45, min(45, turn_rate))

        # Drive with correction
        alvik.drive(BASE_SPEED, turn_rate)

        time.sleep(0.05)
```

This uses **proportional control** to smoothly adjust the robot's heading based on how far it deviates from the ideal wall distance.

---

## Part 3: State Machine Approach

For more complex mazes, a state machine provides better control:

```python
class MazeNavigator:
    def __init__(self):
        self.alvik = ArduinoAlvik()
        self.alvik.begin()
        self.state = "FOLLOW_WALL"

    def run(self):
        while True:
            if self.state == "FOLLOW_WALL":
                self.follow_wall()
            elif self.state == "TURN_LEFT":
                self.turn_left()
            elif self.state == "TURN_RIGHT":
                self.turn_right()
            elif self.state == "MOVE_FORWARD":
                self.move_forward()

    def follow_wall(self):
        left = self.alvik.get_distance_left()
        front = self.alvik.get_distance_center()

        if front < FRONT_OBSTACLE:
            self.state = "TURN_RIGHT"
        elif left > TOO_FAR:
            self.state = "TURN_LEFT"
        else:
            self.state = "MOVE_FORWARD"

    def turn_left(self):
        self.alvik.rotate(-90)
        self.state = "FOLLOW_WALL"

    def turn_right(self):
        self.alvik.rotate(90)
        self.state = "FOLLOW_WALL"

    def move_forward(self):
        self.alvik.drive(10, 0)
        time.sleep(0.1)
        self.state = "FOLLOW_WALL"
```

---

## Part 4: ROS2 Integration

Now let's level up with ROS2! This enables SLAM and autonomous navigation.

### Why ROS2?

ROS2 (Robot Operating System 2) provides:
- **Standard tools** for mapping and navigation
- **Visualization** with RViz
- **Path planning** algorithms
- **Sensor fusion** capabilities
- **Community packages** for robotics tasks

### Setting Up ROS2 on Alvik

First, ensure your Alvik can communicate with your ROS2 computer:

```python
from arduino_alvik import ArduinoAlvik
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class AlvikROS2Node(Node):
    def __init__(self):
        super().__init__('alvik_node')

        # Initialize Alvik
        self.alvik = ArduinoAlvik()
        self.alvik.begin()

        # Create publishers
        self.scan_pub = self.create_publisher(
            LaserScan,
            'scan',
            10
        )

        # Create subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Timer for sensor publishing
        self.timer = self.create_timer(0.1, self.publish_sensors)

    def publish_sensors(self):
        """Publish sensor data as LaserScan"""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'

        # Get distance readings
        distances = [
            self.alvik.get_distance_left(),
            self.alvik.get_distance_center(),
            self.alvik.get_distance_right()
        ]

        scan.ranges = distances
        scan.angle_min = -0.785  # -45 degrees
        scan.angle_max = 0.785   # 45 degrees
        scan.angle_increment = 0.785  # 45 degrees
        scan.range_min = 0.05
        scan.range_max = 3.5

        self.scan_pub.publish(scan)

    def cmd_vel_callback(self, msg):
        """Handle movement commands from ROS2"""
        linear = msg.linear.x * 100  # Convert to cm/s
        angular = msg.angular.z * 57.3  # Convert to degrees/s

        self.alvik.drive(linear, angular)

def main():
    rclpy.init()
    node = AlvikROS2Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.alvik.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Part 5: SLAM with ROS2

With ROS2 integration, we can use powerful SLAM algorithms!

### Installing SLAM Toolbox

```bash
sudo apt install ros-humble-slam-toolbox
```

### Launch SLAM

Create a launch file `alvik_slam.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'base_frame': 'base_link'},
                {'odom_frame': 'odom'},
                {'map_frame': 'map'}
            ]
        ),

        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/path/to/alvik_slam.rviz']
        )
    ])
```

Launch it with:

```bash
ros2 launch alvik_slam.launch.py
```

### Building Your Map

1. **Start SLAM** - Launch the SLAM toolbox
2. **Drive Manually** - Use teleop or joystick control
3. **Watch RViz** - See the map build in real-time
4. **Save Map** - Once complete, save your map

```bash
ros2 run nav2_map_server map_saver_cli -f my_maze
```

---

## Part 6: Autonomous Navigation

With a saved map, Alvik can navigate autonomously!

### Setup Nav2

Install the navigation stack:

```bash
sudo apt install ros-humble-navigation2
```

### Launch Autonomous Navigation

```bash
ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=False \
    map:=/path/to/my_maze.yaml
```

### Send Navigation Goals

In RViz, use the "2D Goal Pose" tool to:
1. Click the destination
2. Drag to set orientation
3. Watch Alvik navigate autonomously!

---

## Comparing the Approaches

| Approach | Complexity | Map Required | Best For |
|----------|-----------|--------------|----------|
| Wall Following | Low | No | Simple mazes, learning |
| State Machine | Medium | No | More complex logic |
| ROS2 + SLAM | High | Builds own | Unknown environments |
| ROS2 + Nav2 | High | Yes | Known environments |
{:class="table table-striped"}

---

## Tips for Success

### Maze Design

1. **Start Simple** - Begin with a simple square maze
2. **Clear Walls** - Ensure walls are easily detectable
3. **Good Lighting** - Time-of-flight sensors need good light
4. **Flat Surface** - Keep the floor level

### Tuning Parameters

**Wall Following:**
- Adjust `WALL_DISTANCE` for your maze width
- Tune `KP` for smoother or more aggressive turning
- Modify `BASE_SPEED` for stability vs. speed

**ROS2 SLAM:**
- Adjust `resolution` for map detail
- Tune `laser_scan_matcher` for accuracy
- Configure `loop_closure` parameters

### Troubleshooting

**Robot gets stuck in corners:**
- Add timeout logic to back up
- Reduce wall following distance
- Implement corner detection

**SLAM map is distorted:**
- Improve odometry calibration
- Reduce driving speed
- Add more sensor data

**Nav2 paths are erratic:**
- Tune costmap parameters
- Adjust planner settings
- Check TF tree for errors

---

## Taking It Further

Once you've mastered maze navigation, try these challenges:

1. **Speed Runs** - Optimize for fastest completion
2. **Dead Reckoning** - Navigate without sensors
3. **Multi-Robot** - Race multiple Alviks
4. **Dynamic Obstacles** - Handle moving obstacles
5. **Full Autonomy** - Start to finish without intervention

---

## Code Repository

All the code from this tutorial is available at:
[github.com/kevinmcaleer/alvik_maze](https://www.github.com/kevinmcaleer/alvik_maze)

The repository includes:
- Basic wall following examples
- Enhanced proportional control
- State machine implementation
- ROS2 integration code
- SLAM configuration files
- Navigation launch files

---

## Resources

- [Arduino Alvik Documentation](https://docs.arduino.cc/hardware/alvik/)
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2 Documentation](https://navigation.ros.org/)
- [My Alvik Code Repository](https://www.github.com/kevinmcaleer/alvik_maze)

---

## Conclusion

Maze navigation is a fantastic way to learn robotics fundamentals! We've progressed from simple reactive behaviors with wall following, through state machines for better control, all the way to sophisticated SLAM and autonomous navigation with ROS2.

The Arduino Alvik is an excellent platform for this journey - it has the sensors, processing power, and connectivity to handle everything from beginner to advanced navigation tasks.

What maze will you conquer first? Share your Alvik navigation projects in the comments!

Happy navigating!

---
