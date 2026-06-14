---
layout: lesson
title: From Hand-Built to Standard
author: Kevin McAleer
type: page
cover: assets/cover.jpg
date: 2026-06-14
previous: 00_intro.html
next: 02_first_environment.html
description: Understand what a standard environment API buys you, and learn the history
  of OpenAI Gym and its maintained successor Gymnasium.
percent: 12
duration: 4
date_updated: 2026-06-14
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
    - name: "What You Have Built \u2014 and Where to Go Next"
      link: 14_summary.html
---


![Cover](assets/cover.jpg){:class="cover"}

---

## What You Built Last Time

In the previous course you wrote a `GridWorld` class with three methods:

```python
world = GridWorld()
state = world.reset()
state, reward, done = world.step(action)
```

That was a deliberate choice. By coding every line yourself — the obstacle check, the sensor, the reward function — you could see exactly where the numbers came from. Nothing was hidden.

The downside is that your code and your agent are tied to each other. If you wanted to test a different algorithm, you would need to rewrite the training loop. If you wanted to swap the grid-world for a simulated arm, you would need to rewrite the environment too. The algorithm and the environment are entangled.

## What a Standard API Buys You

A standard API is a contract: as long as both sides follow the same rules, they can be swapped independently.

With Gymnasium, the contract looks like this:

```python
obs, info = env.reset()
obs, reward, terminated, truncated, info = env.step(action)
```

Any environment that implements those two methods — plus an `observation_space` and an `action_space` — can be used with any agent that expects the same interface. That means:

- You can train DQN on CartPole today and on your BurgerBot environment tomorrow by changing one line.
- You can swap from DQN to PPO without touching the environment at all.
- You can publish your environment and anyone in the community can train their own agents on it.
- Libraries like Stable-Baselines3 work on your environment automatically, without modification.

This is the core value of a standard API. It decouples the problem (the environment) from the solution (the algorithm).

## A Brief History: Gym to Gymnasium

OpenAI released Gym in 2016 as an open-source toolkit for developing and comparing RL algorithms. It became the de facto standard: if you searched for an RL tutorial, it almost certainly used `import gym`.

In 2022, OpenAI shifted focus away from Gym and stopped actively maintaining it. The final release (0.26) introduced breaking changes — including the `terminated`/`truncated` split you will learn in the next lesson — then development largely stopped.

The [Farama Foundation](https://farama.org/), a non-profit organisation focused on open RL tooling, forked Gym and continued it as Gymnasium. Gymnasium 1.0 was released in late 2024. It is the maintained, actively developed successor.

> **Note:** If you see older tutorials using `import gym` and `done = env.step(action)[2]`, they are using the unmaintained OpenAI Gym. The API is similar but not identical. This course uses Gymnasium throughout.

## What Gymnasium Is (and Is Not)

Gymnasium handles the environment API only. It defines the interface and ships a set of classic benchmark environments (CartPole, MountainCar, LunarLander, and others).

Gymnasium does NOT include:

- Physics engines (MuJoCo, PyBullet, and Isaac Gym are separate projects)
- Learning algorithms (Stable-Baselines3, RLlib, CleanRL, and others provide those)
- Multi-agent support (PettingZoo, also by Farama, handles that)

Think of Gymnasium as the socket and the plug standard. It does not do the computing; it makes sure every piece fits together.

## The Farama Ecosystem

Farama maintains a growing family of tools that all speak the same API:

- **Gymnasium** — the core environment API and classic benchmarks
- **PettingZoo** — multi-agent environments
- **Minari** — offline RL datasets (recorded trajectories from trained agents)
- **Shimmy** — compatibility shims for third-party envs (MuJoCo, DM Control, Atari)

You can build an environment today, share it via Minari, and let the community train agents on it. The standard API is what makes that possible.

## Try It Yourself

Before moving on, think about the `GridWorld` class you built in the previous course.

1. Which parts of the `GridWorld` class would change if you swapped the 7x7 BurgerBot grid for a 10x10 maze? List them.
2. Which parts of the Q-learning training loop would need to change? (Hint: probably fewer than you think.)
3. Look at the Gymnasium environment list at [gymnasium.farama.org](https://gymnasium.farama.org). Find one environment that sounds interesting. What is its observation space and action space?

The next lesson will get your hands on the actual API.
