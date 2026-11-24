---
layout: course_overview
title: Reinforcement Learning with Gymnasium
author: Kevin McAleer
cover: /learn/gymnasium/assets/cover.jpg
date: 2026-06-14
description: Take your Q-learning skills to the industry-standard Gymnasium API and
  Stable-Baselines3. You will wrap the BurgerBot grid-world as a proper Gymnasium
  environment, discretize CartPole with your own binning code, and train neural-network
  agents that export a policy table you can drop straight onto a Pico.
level: intermediate
course_duration: 1h 44m
lesson_count: 15
first_lesson: 00_intro.html
quiz_total: 56
video: epuO7xpz6i4
outcomes:
- Use the current Gymnasium API (reset, step with terminated and truncated, spaces,
  seeding) with standard environments
- Explain why terminated and truncated are separate and use each correctly in a Q-learning
  update
- Write a custom gym.Env for the BurgerBot grid-world, check it with check_env and
  register it with gym.make
- Add behaviour with wrappers and run several environment copies with vector environments
- Train, evaluate, save and load a Stable-Baselines3 PPO or DQN agent on your own
  environment
- Export a trained policy as a JSON lookup table and say honestly what it can and
  cannot do on a Pico
prerequisites:
- The Reinforcement Learning for Beginners course (Q-tables, epsilon-greedy, the BurgerBot
  grid-world)
- Python basics such as functions, dictionaries and classes, for example the Introduction
  to Python course
you_need:
- A computer running macOS, Linux or Windows with Python 3.10 or newer and a virtual
  environment
- pip install "gymnasium[classic-control]" stable-baselines3 (PyTorch comes with Stable-Baselines3
  and is a large download)
- No robot or other hardware, and no GPU. Tested with Gymnasium 1.4.0 and Stable-Baselines3
  2.9.2
navigation:
- name: Reinforcement Learning with Gymnasium
- content:
  - section: Introduction
    content:
    - name: Reinforcement Learning with Gymnasium
      link: 00_intro.html
  - section: Why Gymnasium
    content:
    - name: From Hand-Built to Standard
      link: 01_from_handbuilt_to_standard.html
    - name: Your First Gymnasium Environment
      link: 02_first_environment.html
    - name: Terminated vs Truncated
      link: 03_terminated_vs_truncated.html
  - section: The Gymnasium API in Depth
    content:
    - name: Observation Spaces and Action Spaces
      link: 04_spaces.html
    - name: Q-Learning on Gymnasium
      link: 05_qlearning_on_gymnasium.html
    - name: Seeding and the Episode Lifecycle
      link: 06_seeding_and_lifecycle.html
  - section: Building Your Own Environment
    content:
    - name: Anatomy of a Custom Environment
      link: 07_anatomy_of_custom_env.html
    - name: Wrapping the BurgerBot
      link: 08_wrapping_the_burgerbot.html
    - name: Registering and Rendering
      link: 09_registering_and_rendering.html
    - name: Gymnasium Wrappers
      link: 10_wrappers.html
  - section: Scaling Up to Neural Networks
    content:
    - name: Vectorised Environments
      link: 11_vectorized_envs.html
    - name: Meet Stable-Baselines3
      link: 12_meet_stable_baselines3.html
    - name: Train, Evaluate, and Export
      link: 13_train_evaluate_export.html
    - name: What You Have Built, and Where to Go Next
      link: 14_summary.html
---
