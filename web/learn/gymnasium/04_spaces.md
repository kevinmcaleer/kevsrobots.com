---
layout: lesson
title: Observation Spaces and Action Spaces
author: Kevin McAleer
type: page
cover: assets/cover.jpg
date: 2026-06-14
previous: 03_terminated_vs_truncated.html
next: 05_qlearning_on_gymnasium.html
description: "Learn how Gymnasium's Discrete, Box, and MultiDiscrete spaces describe\
  \ what an agent can observe and do \u2014 and why spaces make generic algorithms\
  \ possible."
percent: 30
duration: 5
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

## Why Spaces Exist

When you wrote the BurgerBot Q-learning agent by hand, your `choose_action` function knew the exact structure of the state: a tuple of `(row, col, heading, sensor)`. It knew that `row` ranged from 0 to 6, that `heading` was one of four string values, and that `sensor` had three levels. You hard-coded that knowledge.

A generic algorithm — one that should work on CartPole, BurgerBot, and a simulated robot arm without modification — cannot have that knowledge baked in. It needs a way to ask: "What does the observation look like? What actions are available?"

That is what `observation_space` and `action_space` are for. They are machine-readable descriptions of the shape and range of the data the environment produces and accepts.

## The Three Spaces You Will Use

Gymnasium defines many space types. Three cover the vast majority of environments you will encounter.

### Discrete

A single integer chosen from `{0, 1, 2, ..., n-1}`.

```python
from gymnasium import spaces

# Four possible actions: 0, 1, 2, 3
action_space = spaces.Discrete(4)

print(action_space.n)           # 4
print(action_space.sample())    # random integer in {0, 1, 2, 3}
print(action_space.contains(2)) # True
print(action_space.contains(4)) # False — 4 is out of range
```

BurgerBot uses `Discrete(4)` for actions: 0=forward, 1=turn_left, 2=turn_right, 3=stop.

### Box

A continuous n-dimensional array, where each dimension has its own lower and upper bound.

```python
import numpy as np
from gymnasium import spaces

# CartPole's observation: 4 continuous values
# [cart_position, cart_velocity, pole_angle, pole_angular_velocity]
obs_space = spaces.Box(
    low=np.array([-4.8, -np.inf, -0.418, -np.inf]),
    high=np.array([4.8, np.inf, 0.418, np.inf]),
    dtype=np.float32
)

print(obs_space.shape)          # (4,)
print(obs_space.sample())       # random float array — will be in range
print(obs_space.contains(np.zeros(4, dtype=np.float32)))  # True
```

Box is what you get with continuous sensor data, camera images (a Box of shape `(H, W, 3)`), joint angles, velocities, and most real-world robotics observations.

### MultiDiscrete

An array of independent integers, each from its own range. This is what BurgerBot uses for observations.

```python
from gymnasium import spaces

# BurgerBot observation: [row, col, heading_index, sensor_reading]
# row: 0-6, col: 0-6, heading: 0-3, sensor: 0-2
obs_space = spaces.MultiDiscrete([7, 7, 4, 3])

print(obs_space.nvec)           # array([7, 7, 4, 3])
print(obs_space.sample())       # e.g. array([3, 5, 1, 2])
print(obs_space.contains(np.array([6, 6, 3, 2])))   # True
print(obs_space.contains(np.array([7, 0, 0, 0])))   # False — row 7 out of range
```

The total number of possible observations is `7 * 7 * 4 * 3 = 588`. This matches exactly what the previous course calculated when discussing the state-space size.

## How a Generic Algorithm Uses Spaces

Here is a simplified version of what Stable-Baselines3 does internally when you call `model = DQN("MlpPolicy", env)`:

```python
# The algorithm inspects the spaces without knowing what the environment is.
n_actions = env.action_space.n                  # 4 for BurgerBot
obs_shape = env.observation_space.shape         # (4,) for CartPole

# It builds a neural network whose input size matches the observation shape
# and whose output size matches the number of actions.
# It never reads the actual environment code.
```

This is the power of the contract. The algorithm does not know if it is training on CartPole or BurgerBot. It only knows the shape of the data coming in and the range of actions going out.

## Checking What an Environment Has

You can always inspect any Gymnasium environment's spaces:

```python
import gymnasium as gym

env = gym.make("CartPole-v1")
print("Observation space:", env.observation_space)
print("Action space:     ", env.action_space)
print("Obs shape:        ", env.observation_space.shape)
print("Obs dtype:        ", env.observation_space.dtype)
print("Num actions:      ", env.action_space.n)
env.close()
```

Output:

```
Observation space: Box([-4.8       -inf -0.4188    -inf], [4.8        inf  0.4188     inf], (4,), float32)
Action space:      Discrete(2)
Obs shape:         (4,)
Obs dtype:         float32
Num actions:       2
```

## Using .sample() in Training Loops

`space.sample()` is used in two places in RL code:

1. Random agents and baselines: `action = env.action_space.sample()` gives you a legal random action without knowing the environment's internals.
2. Exploration during epsilon-greedy: when the random roll is below epsilon, you call `env.action_space.sample()` instead of querying the Q-table.

```python
import random

epsilon = 0.3

if random.random() < epsilon:
    # Explore: pick a random legal action
    action = env.action_space.sample()
else:
    # Exploit: pick the best known action
    action = best_action_from_q_table(state)
```

This replaces the `random.choice(ACTIONS)` call from the previous course with something that works on any environment, regardless of how many actions it has.

## Try It Yourself

1. Create a `Box` space for a 64x64 greyscale image observation: values between 0 and 255, `dtype=np.uint8`. What is `space.shape`? How many possible observations exist? (You don't need to compute the exact number — just express it symbolically.)
2. Create a `MultiDiscrete([3, 3])` space representing a 3x3 grid position. How many distinct positions can the agent be in?
3. Inspect the `LunarLander-v3` environment's spaces (use `gym.make("LunarLander-v3")`). Is the action space discrete or continuous? How many observations does the agent get?

## Common Issues

**Problem:** `AssertionError: observation is not in observation_space` from `check_env`
**Solution:** Your `reset()` or `step()` returned an observation with the wrong dtype or out-of-range values.
**Why:** NumPy arrays have strict dtypes. `np.array([1, 2, 3])` defaults to `int64`, but `MultiDiscrete` expects `int64` too — usually fine. `Box` with `dtype=float32` rejects a `float64` array. Use `dtype=np.int64` or `dtype=np.float32` explicitly in your environment.

**Problem:** `AttributeError: 'Box' object has no attribute 'n'`
**Solution:** `.n` is a `Discrete` attribute. Use `env.action_space.shape` or `env.action_space.low` for Box spaces.
**Why:** Different space types have different attributes. Always check the space type before accessing type-specific properties.
