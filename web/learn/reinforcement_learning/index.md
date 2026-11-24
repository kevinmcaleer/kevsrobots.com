---
layout: course_overview
title: Reinforcement Learning for Beginners
author: Kevin McAleer
cover: /learn/reinforcement_learning/assets/cover.jpg
date: 2026-06-13
description: Teach your robot to learn — instead of hardcoding every behaviour, use
  tabular Q-learning to let it discover through trial and error how to navigate obstacles
  on its own. Train a policy in a pure-Python grid-world simulation on your laptop,
  then export the frozen Q-table as JSON and deploy it to a Pico-powered differential-drive
  robot running MicroPython.
level: beginner
course_duration: 1h 57m
lesson_count: 16
first_lesson: 00_intro.html
quiz_total: 45
video: epuO7xpz6i4
outcomes:
- Explain the reinforcement learning loop (agent, environment, state, action, reward)
  using a robot as the example
- Design a reward function and spot reward hacking before it wastes a training run
- Write tabular Q-learning from scratch in plain Python, using epsilon-greedy exploration
- Train a policy in a grid-world simulation, read its learning curve and test it with
  sensor noise
- Load a trained Q-table onto a Raspberry Pi Pico and run it as a simple obstacle-avoiding
  robot
- Say honestly what a toy Q-table does and does not tell you about real robot learning
prerequisites:
- Python basics such as functions, loops, dictionaries and classes, for example the
  Introduction to Python course
- For the last three lessons, MicroPython motor and ultrasonic sensor basics, for
  example the MicroPython Robotics Projects course
you_need:
- A computer with a recent Python 3 for lessons 1 to 11 (standard library only, no
  installs)
- Lessons 12 and 13 only, for the real robot, a Raspberry Pi Pico with MicroPython,
  an L298N motor driver, two DC gear motors, an HC-SR04 ultrasonic sensor with two
  resistors (1k and 2k), and a battery pack
- Everything except the last two lessons works with no hardware at all
navigation:
- name: Reinforcement Learning for Beginners
- content:
  - section: Introduction
    content:
    - name: Introduction to Reinforcement Learning for Beginners
      link: 00_intro.html
  - section: The Paradigm Shift
    content:
    - name: Beyond Predefined Steps
      link: 01_beyond_predefined_steps.html
    - name: The Anatomy of Robotic RL
      link: 02_anatomy_of_robotic_rl.html
    - name: Designing the Reward Function
      link: 03_designing_the_reward_function.html
  - section: The Framework of Decisions
    content:
    - name: Mapping the Robot's World
      link: 04_mapping_the_robots_world.html
    - name: Markov Decision Processes
      link: 05_markov_decision_processes.html
    - name: Looking Into the Future
      link: 06_looking_into_the_future.html
  - section: Tabular Q-Learning from Scratch
    content:
    - name: The Q-Table
      link: 07_the_q_table.html
    - name: Exploration vs Exploitation
      link: 08_exploration_vs_exploitation.html
    - name: Coding the Learning Loop
      link: 09_coding_the_learning_loop.html
    - name: Training in Simulation
      link: 10_training_in_simulation.html
  - section: Bringing It to the Real World
    content:
    - name: The Sim-to-Real Gap
      link: 11_the_sim_to_real_gap.html
    - name: Discretising Real Sensors
      link: 12_discretizing_real_sensors.html
    - name: Deploying the Brain
      link: 13_deploying_the_brain.html
    - name: Scaling Up to Deep RL
      link: 14_scaling_up_to_deep_rl.html
    - name: Summary and What's Next
      link: 15_summary.html
---
