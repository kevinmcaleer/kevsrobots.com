---
layout: lesson
title: From Wall Following to Mapping and SLAM
author: Kevin McAleer
type: page
cover: /learn/robot_senses/assets/banners/14_mapping_and_slam.jpg
date: 2026-11-24
previous: 13_wall_following.html
next: 15_troubleshooting.html
description: See where wall following runs out, then learn the ideas behind maps,
  odometry drift, SLAM and navigation using the Arduino Alvik maze project and ROS2
percent: 75
duration: 7
quiz_total: 10
date_updated: 2026-11-24
navigation:
- name: Give Your Robot Senses
- content:
  - section: Introduction
    content:
    - name: Introduction
      link: 00_intro.html
  - section: How robots sense
    content:
    - name: How Robots Sense
      link: 01_how_robots_sense.html
    - name: Meet Modulino SMARS
      link: 02_meet_modulino_smars.html
    - name: Qwiic and I2C
      link: 03_qwiic_and_i2c.html
    - name: Your First Connection
      link: 04_first_connection.html
  - section: One sense at a time
    content:
    - name: See with the Distance Module
      link: 05_see_distance.html
    - name: Feel Movement with the IMU
      link: 06_feel_movement.html
    - name: Speak with the Buzzer
      link: 07_speak_the_buzzer.html
    - name: Show with the LED Matrix
      link: 08_show_the_led_matrix.html
    - name: Move with the Motors
      link: 09_move_the_motors.html
  - section: From readings to behaviour
    content:
    - name: From Readings to Decisions
      link: 10_readings_to_decisions.html
    - name: Combining Senses
      link: 11_combining_senses.html
    - name: 'Project: The Reactive Robot'
      link: 12_project_reactive_robot.html
  - section: Navigation
    content:
    - name: Wall Following
      link: 13_wall_following.html
    - name: From Wall Following to Mapping and SLAM
      link: 14_mapping_and_slam.html
  - section: Wrapping up
    content:
    - name: Troubleshooting
      link: 15_troubleshooting.html
    - name: Summary
      link: 16_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

Wall following is clever, but it never knows where it is. It has no memory of the maze, so it can't take a shortcut, and it can't escape a maze with islands. To do better, a robot needs a **map**, and it needs to know where it is on that map. This lesson is about the ideas. The real thing needs a computer running ROS2, so we'll treat it as a tour, not a build.

Kev's Alvik maze project climbs this whole ladder, from wall following to SLAM. Here is the shape of it.

## The ladder

| Approach | Complexity | Map required | Best for |
|---|---|---|---|
| Wall following | Low | No | Simple mazes, learning |
| State machine | Medium | No | More complex logic |
| ROS2 and SLAM | High | Builds its own | Unknown environments |
| ROS2 and Nav2 | High | Yes | Known environments |
{:class="table table-single"}

You've done the first row. Let's climb.

## Step up: a state machine

Lesson 10 introduced states. Kev's maze post uses the same idea for navigation: states such as `FOLLOW_WALL`, `TURN_LEFT`, `TURN_RIGHT` and `MOVE_FORWARD`. Each state does one thing, and the follow-wall state decides which state comes next based on the left and front distances. It's the same behaviour as lesson 13, but organised so you can add new states, such as backing out of a corner, without tangling the code.

## What a map is

For a robot, a map can be a simple grid. Each cell is either **unknown**, **free** or **occupied**. When the distance sensor says "there's a wall 4 cells ahead", the robot learns two things: the three cells in front of it are free, and the fourth is occupied.

```python
WIDTH, HEIGHT = 10, 5
UNKNOWN, FREE, WALL = "?", ".", "#"

grid = [[UNKNOWN] * WIDTH for _ in range(HEIGHT)]

def add_reading(x, y, cells_ahead):
    """The robot is at (x, y) facing east and sees a wall cells_ahead away."""
    for step in range(1, cells_ahead):
        grid[y][x + step] = FREE       # the beam passed through these
    grid[y][x + cells_ahead] = WALL    # the beam stopped here

# The robot drives east along row 2 and takes three readings
add_reading(1, 2, 6)
add_reading(2, 2, 5)
add_reading(3, 2, 4)

for row in grid:
    print("".join(row))
```

Run it and you'll see a corridor of free cells ending in a wall, surrounded by unknown cells. Add more readings from more places and the map fills in. That's mapping in its simplest form. Notice that the code assumes the robot knows exactly where it is, and that is the hard part.

## Why knowing where you are is hard

A robot estimates its position by counting its own movement. That's called **odometry**. But wheels slip, motors differ and tracks skid. Every small error adds to the last. Here is a robot that thinks it's driving straight, but has a tiny heading error that grows each step:

```python
from math import sin, cos, radians

STEP_CM = 10
DRIFT_DEG_PER_STEP = 0.5     # a small, constant slip

true_x, true_y, heading = 0.0, 0.0, 0.0
believed_x = 0.0

for step in range(1, 41):
    heading = heading + DRIFT_DEG_PER_STEP          # slip we can't see
    true_x = true_x + STEP_CM * cos(radians(heading))
    true_y = true_y + STEP_CM * sin(radians(heading))
    believed_x = believed_x + STEP_CM                # what odometry says

    if step % 10 == 0:
        error = ((true_x - believed_x) ** 2 + true_y ** 2) ** 0.5
        print("step", step, "believes x =", believed_x, "but is off by",
              round(error, 1), "cm")
```

The error isn't constant. It grows the longer the robot drives. A map built on that position would smear, bend and double up. This is exactly the problem you read about with the gyroscope in lesson 6, where integrating a rate slowly drifts. Kev's tip for a distorted SLAM map is the same: improve your odometry calibration, drive slower, and give it more sensor data.

## SLAM in one sentence

**SLAM** means Simultaneous Localization and Mapping. The robot builds the map and works out where it is on that map at the same time. Each new reading is matched against the map so far, which corrects the drift. A robot that recognises a place it has been before can pull the whole map back into shape. That's called loop closure, and it's the reason SLAM maps stay straight.

## Where ROS2 comes in

Writing SLAM yourself would take months. **ROS2**, the Robot Operating System, is a toolbox that already has it. According to Kev's post it gives you:

- Standard tools for mapping and navigation
- Visualisation with RViz
- Path planning algorithms
- Sensor fusion
- Community packages for robotics tasks

In the post, the Alvik connects to a ROS2 computer over WiFi. A Python program on the Alvik publishes its sensor readings as a `LaserScan` message called `scan`, and listens for velocity commands on `cmd_vel`. The Alvik only has three distance readings, left, centre and right, so the scan is very sparse. A scanning lidar gives many more readings per turn. Kev's [LidarBot](https://github.com/kevinmcaleer/lidarbot) uses an LD06 360 degree lidar for 2D mapping, which is the sort of sensor SLAM is happiest with.

The post installs the toolbox and runs it on the ROS2 computer:

```bash
sudo apt install ros-humble-slam-toolbox
ros2 launch alvik_slam.launch.py
```

You then drive the robot around by hand with teleop or a joystick, and watch the map grow in RViz. When it looks right, save it:

```bash
ros2 run nav2_map_server map_saver_cli -f my_maze
```

## Navigating a known map

With a saved map, the robot can plan. The Nav2 navigation stack lets you click a destination in RViz with the "2D Goal Pose" tool, drag to set the direction, and watch the robot find its own way there. Wall following can never do that, because it has no idea where the goal is.

The full ROS2 code and launch files are in Kev's [alvik_maze repository](https://github.com/kevinmcaleer/alvik_maze). The install commands above are for the ROS2 version the post used, so check the current ROS2 and Nav2 documentation before you copy them. Our [Learn ROS](/learn/learn_ros/) course is the place to build the ROS2 skills.

## Try it Yourself

1. In the grid code, add a fourth reading from `(4, 2)` with `cells_ahead` of 3. What changes?
2. Make the robot turn: add a version of `add_reading()` that faces south instead of east.
3. In the drift code, halve `DRIFT_DEG_PER_STEP`. How does the error at step 40 change?
4. Challenge: print the error at every step and spot how it speeds up.

## Common Issues

**Problem**: The SLAM map is distorted.
**Solution**: Calibrate the odometry, drive slower, and add more sensor data.
**Why**: Errors in position accumulate, so each reading is placed a little wrong.

**Problem**: The robot's paths in Nav2 are erratic.
**Solution**: Tune the costmap and planner settings, and check the TF tree for errors.
**Why**: The planner works from the map and the transforms. Wrong values in either make wild plans.

**Problem**: The robot gets stuck in corners.
**Solution**: Add timeout logic to back up, and reduce the wall following distance.
**Why**: Reactive rules have no memory of where they've been.
