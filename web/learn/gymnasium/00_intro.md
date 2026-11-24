---
layout: lesson
title: Reinforcement Learning with Gymnasium
author: Kevin McAleer
type: page
cover: /learn/gymnasium/assets/banners/00_intro.jpg
date: 2026-06-14
next: 01_from_handbuilt_to_standard.html
description: Meet the industry-standard Gymnasium API, explore Stable-Baselines3,
  and train neural-network agents on the BurgerBot grid-world you already know.
percent: 6
duration: 4
quiz_total: 56
snakie: false
date_updated: 2026-11-24
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


![Course Cover Image]({{page.cover}}){:class="cover"}

---

Watch this first: a MuJoCo pendulum learns to balance with Stable-Baselines3 and a reward, the same tools this course uses.

{% include youtubeplayer.html id="epuO7xpz6i4" %}

---

## Overview

In the [Reinforcement Learning for Beginners](/learn/reinforcement_learning/) course you built everything by hand: a `GridWorld` class, a Q-table stored in a plain Python dictionary, an epsilon-greedy action selector and a reward function tuned for the BurgerBot maze. That was the right place to start, because understanding every line of a hand-rolled environment is how the ideas stick.

This course ports those same ideas onto the tools that researchers and industry teams use. **Gymnasium**, maintained by the Farama Foundation, is the standard API for describing environments. **Stable-Baselines3** (SB3) is a library of ready-made agents, including DQN and PPO, that plug into any Gymnasium environment. Together they let you swap algorithms, share code with the community, and move on to problems where a lookup table runs out of road.

The through-line project is the BurgerBot grid-world you already know. By the end you will have wrapped it as a proper Gymnasium environment, checked it with the official checker, trained a neural-network policy on it, and exported that policy as a JSON lookup table.

```mermaid
flowchart LR
    A["Hand-built GridWorld<br/>(previous course)"] --> B["Gymnasium API<br/>reset and step"]
    B --> C["Custom BurgerBotEnv<br/>checked and registered"]
    C --> D["Wrappers and<br/>vector envs"]
    D --> E["Stable-Baselines3<br/>PPO agent"]
    E --> F["nn_policy.json<br/>lookup table"]
```

## Prerequisites

- [Reinforcement Learning for Beginners](/learn/reinforcement_learning/): you need Q-tables, epsilon-greedy exploration and the BurgerBot grid-world. This course builds on those ideas and does not re-teach them.
- [Introduction to Python](/learn/python/): functions, dictionaries, f-strings and simple classes.

## Course Content

- What the Gymnasium API is, and why it replaced OpenAI Gym
- Installing Gymnasium and running your first environment
- The five values returned by `env.step()` and why `terminated` and `truncated` are separate
- Observation spaces and action spaces: `Discrete`, `Box`, `MultiDiscrete`
- Running your tabular Q-learning agent on CartPole
- Seeding, the episode lifecycle and `env.close()`
- The `gym.Env` contract, and a custom environment that passes `check_env`
- Wrapping BurgerBot, registering it with `gym.register` and rendering it as text
- Wrappers, and vectorised environments
- Training DQN and PPO with Stable-Baselines3, then saving, loading and evaluating
- Exporting a neural-network policy as a lookup table

## Key Results

After completing this course, you will be able to:

- Use the Gymnasium API to interact with standard environments, with the current return values
- Explain the difference between `terminated` and `truncated` and why it matters for learning
- Write a custom `gym.Env` that passes `check_env`, and register it with `gym.make`
- Train a Stable-Baselines3 agent on your own environment in about twenty lines
- Export a trained policy to a JSON table, and say honestly what that table can and cannot do on a Pico

## What you'll need

This course runs entirely on a normal computer. There is no hardware to build.

- A laptop or desktop running macOS, Linux or Windows
- Python 3.10 or newer
- A virtual environment and `pip install "gymnasium[classic-control]" stable-baselines3` (the next lesson walks through it)
- Free disk space for PyTorch, which SB3 installs for you. On Linux the default download includes NVIDIA GPU libraries and took about 5 GB in testing. A GPU is not needed: everything here trains on a CPU in under a minute.

## How the course works

Every program in this course was run before it was published, with fixed random seeds, using **Python 3.13, Gymnasium 1.4.0 and Stable-Baselines3 2.9.2**. The output shown is what we saw. Where a number depends on random training, we say so, and your numbers may differ a little on other machines.

Programs are complete: save them, run them, change them. From lesson 8 onwards we keep the BurgerBot environment in one file called `burgerbot_env.py`, and later programs import it from there, so keep all your files in one folder.

Each lesson opens with an objective and ends with **Try it Yourself** exercises, **Common Issues** and a short quiz. Notes appear as blockquotes. The `> **Teachers**:` notes cover classroom catches.

> **Teachers**: The course suits learners aged about 14 and up who have finished the Reinforcement Learning for Beginners course and are comfortable with Python classes. Allow 30 to 45 minutes per lesson, and a double period for lessons 8, 12 and 13. Each learner or pair needs one computer with Python and an internet connection for the first install. The PyTorch download is large, so install it once in advance on a classroom machine or ask learners to install before the lesson, as many school networks throttle big downloads. No hardware is used. Lessons 1 to 10 need only Gymnasium, which is small, so a class can start before PyTorch arrives.
