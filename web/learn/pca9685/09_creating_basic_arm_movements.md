---
layout: lesson
title: Basic Arm Movements
author: Kevin McAleer
type: page
cover: assets/9.png
date: 2024-04-06
previous: 08_controlling_servos_with_python.html
next: 10_advanced_arm_control_and_automation.html
description: Dive deeper into programming your robot arm with Python, focusing on
  creating basic yet essential movements and sequences.
percent: 72
duration: 3
navigation:
- name: Building a Robot Arm with Raspberry Pi and PCA9685
- content:
  - section: Getting Started with Raspberry Pi and Python
    content:
    - name: Introduction
      link: 01_course_introduction.html
    - name: Setting Up Your Raspberry Pi
      link: 02_raspberry_pi_setup.html
    - name: Python Basics for Robotics
      link: 03_python_basics_for_robotics.html
  - section: Understanding the PCA9685 Servo Driver
    content:
    - name: PCA9685 Servo Driver
      link: 04_introduction_to_pca9685.html
    - name: Setting Up PCA9685
      link: 05_setting_up_the_pca9685_with_raspberry_pi.html
  - section: Building the Robot Arm
    content:
    - name: Design
      link: 06_designing_your_robot_arm.html
    - name: Assembly
      link: 07_assembling_the_robot_arm.html
  - section: Programming the Robot Arm
    content:
    - name: Controlling Servos with Python
      link: 08_controlling_servos_with_python.html
    - name: Basic Arm Movements
      link: 09_creating_basic_arm_movements.html
    - name: Advanced Control and Automation
      link: 10_advanced_arm_control_and_automation.html
  - section: Project Challenges and Ideas
    content:
    - name: Project Ideas
      link: 11_project_ideas.html
    - name: Troubleshooting
      link: 12_troubleshooting_common_issues.html
---


![Creating Basic Arm Movements]({{ page.cover }}){:class="cover"}

## Introduction to Arm Movement Sequences

Now that you've learned the basics of controlling servos with Python, let's apply these skills to create a series of basic yet fundamental movements for your robot arm. This lesson will guide you through programming your robot arm to perform tasks such as reaching out, grasping an object, and retracting.

---

## Planning Your Movements

Before coding, it's essential to plan the sequences of movements your robot arm will perform. Consider the following steps for a simple task like picking up an object:

1. **Reach:** Extend the arm towards the object.
2. **Grasp:** Close the end effector (e.g., a claw or hand) around the object.
3. **Retract:** Bring the object back towards the base.

---

## Example Code: Reaching Out

Let's start by programming our robot arm to reach out. We'll need to adjust the angles of the servos responsible for the arm's extension.

```python
def reach_out():
    set_servo_position(base_channel, base_forward_position)
    set_servo_position(shoulder_channel, shoulder_down_position)
    set_servo_position(elbow_channel, elbow_straight_position)
    time.sleep(2)  # Wait for the arm to fully extend
```

---

## Example Code: Grasping an Object

Programming the end effector to grasp an object involves closing the claw or hand by moving its controlling servo to the required position.

```python
def grasp_object():
    set_servo_position(claw_channel, claw_closed_position)
    time.sleep(1)  # Allow time for the end effector to close
```

---

## Example Code: Retracting the Arm

Finally, retract the arm by reversing the extension movements. Ensure the object is securely grasped before retracting.

```python
def retract_arm():
    set_servo_position(elbow_channel, elbow_bent_position)
    set_servo_position(shoulder_channel, shoulder_up_position)
    set_servo_position(base_channel, base_start_position)
    time.sleep(2)  # Wait for the arm to fully retract
```

---

## Putting It All Together

Combine the functions above into a sequence to perform the complete task.

```python
def perform_task():
    reach_out()
    grasp_object()
    retract_arm()
    # Place additional movements or tasks here

perform_task()
```

---

## Experimenting with Movements

Experiment with the servo positions and timings to refine your robot arm's movements. Each robot arm will be unique due to differences in construction and servo characteristics, so adjustments are expected.

---

## Conclusion

You've now programmed your robot arm to perform basic tasks, marking a significant milestone in your robotics journey. Continue to build upon these foundations by creating more complex sequences and exploring additional functionalities for your robot arm.

---

## Lesson Assignment

Create a sequence that involves moving an object from one location to another. Document the servo positions and timings used, and reflect on the challenges faced and how you overcame them.

---
