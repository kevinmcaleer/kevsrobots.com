---
title: Vectorised Environments
description: Speed up training by running multiple environment copies in parallel with SyncVectorEnv, and understand how batched observations and rewards work.
layout: lesson
type: page
cover: assets/cover.jpg
date_updated: 2026-06-14
---

![Cover](assets/cover.jpg){:class="cover"}

---

## The Speed Problem

Training a neural-network agent on a single environment is slow. The agent generates one transition at a time: one observation, one action, one reward. Generating enough transitions to train a good policy can take tens of thousands of episodes.

The environment itself is usually fast — the BurgerBot grid-world can step a million times per second. The bottleneck is often the ratio of transitions collected to compute spent. Neural-network algorithms like PPO train more efficiently when they see a batch of diverse transitions at each update step, rather than one at a time.

The solution is to run N copies of the environment in parallel and collect N transitions per step. Gymnasium's vectorised environment interface makes this straightforward.

## SyncVectorEnv

`SyncVectorEnv` runs multiple environments in a single process, sequentially. Despite "Sync" in the name, it does give you batched observations and rewards — it just runs the environments one after another in the same thread rather than in parallel threads. For pure-Python environments like BurgerBot, this is actually faster than multi-process approaches because it avoids inter-process serialisation overhead.

```python
import gymnasium as gym
import numpy as np

# Create a vectorised CartPole with 4 parallel environments.
# Each factory function creates one independent environment.
def make_env(seed):
    def _init():
        env = gym.make("CartPole-v1")
        env.reset(seed=seed)
        return env
    return _init

vec_env = gym.vector.SyncVectorEnv([make_env(i) for i in range(4)])

# reset() returns a BATCH of observations: shape (4, 4) for CartPole
# (4 envs, 4-dimensional observation each)
obs, info = vec_env.reset(seed=0)
print("Batch observation shape:", obs.shape)  # (4, 4)

# step() takes a BATCH of actions: shape (4,)
actions = vec_env.action_space.sample()
print("Batch action shape:", actions.shape)   # (4,)

obs, rewards, terminated, truncated, infos = vec_env.step(actions)
print("Batch rewards:", rewards)              # array of 4 floats
print("Terminated:", terminated)             # array of 4 bools

vec_env.close()
```

## How Auto-Reset Works

In a normal environment, you call `env.reset()` when the episode ends. In a vectorised environment, each sub-environment resets itself automatically when its episode terminates or truncates. The observation returned by `vec_env.step()` is always the first observation of the new episode for any environment that just ended.

The previous episode's final observation is stored in `infos["final_observation"]` so you do not lose it:

```python
vec_env = gym.vector.SyncVectorEnv([make_env(i) for i in range(2)])
obs, info = vec_env.reset(seed=0)

episode_counts = np.zeros(2, dtype=int)

for step in range(200):
    actions = vec_env.action_space.sample()
    obs, rewards, terminated, truncated, infos = vec_env.step(actions)
    done = terminated | truncated

    for i, d in enumerate(done):
        if d:
            episode_counts[i] += 1

print(f"Episodes completed per env after 200 steps: {episode_counts}")
vec_env.close()
```

## When Vectorisation Helps

Vectorisation is most valuable for on-policy algorithms like PPO that need a large batch of fresh experience at each update. PPO collects, say, 2048 steps of experience before doing a gradient update. With 4 environments, you collect 2048 steps in 512 calls to `step()` instead of 2048. The diversity of experiences (different starting points, different random outcomes) also tends to reduce variance in the gradient estimate.

For off-policy algorithms like DQN that use a replay buffer, the benefit is smaller but still real — you fill the replay buffer faster.

For the small, fast BurgerBot environment, vectorisation makes less difference than it does for environments with expensive rendering or physics. But understanding the pattern is important because it is how Stable-Baselines3 works internally.

## How SB3 Uses Vectorised Envs

When you call `model = PPO("MlpPolicy", env, n_envs=4)`, Stable-Baselines3 wraps your environment in a vectorised wrapper automatically. Under the hood it is doing exactly what the code above does. You can also pass a pre-built vectorised environment:

```python
from stable_baselines3 import PPO

vec_env = gym.vector.SyncVectorEnv([make_env(i) for i in range(4)])
model = PPO("MlpPolicy", vec_env, verbose=0)
model.learn(total_timesteps=2000)
vec_env.close()
```

> **Note:** Stable-Baselines3 has its own `make_vec_env` helper that handles the factory pattern for you. We will use it in lessons 12 and 13.

## A Vectorised BurgerBot Example

```python
import numpy as np
import gymnasium as gym
from gymnasium import spaces

# (BurgerBotEnv class definition goes here — copy from lesson 08)

def make_burgerbot(seed):
    def _init():
        env = BurgerBotEnv()
        env.reset(seed=seed)
        return env
    return _init

NUM_ENVS = 4
vec_env = gym.vector.SyncVectorEnv([make_burgerbot(i) for i in range(NUM_ENVS)])

obs, info = vec_env.reset(seed=0)
print("Observation batch shape:", obs.shape)   # (4, 4) — 4 envs, 4-dim obs

total_steps = 0
total_goals  = 0

for step in range(500):
    actions = vec_env.action_space.sample()
    obs, rewards, terminated, truncated, infos = vec_env.step(actions)
    total_steps += NUM_ENVS
    # Count goal reaches (reward == 50.0 means goal)
    total_goals += int((rewards == 50.0).sum())

vec_env.close()
print(f"Steps collected: {total_steps}")
print(f"Goal reaches by random agent: {total_goals}")
```

## Try It Yourself

1. Change `NUM_ENVS` to 8 and run 500 steps. Does the number of goal reaches scale proportionally? (It should roughly double.)
2. Try `gym.vector.AsyncVectorEnv` instead of `SyncVectorEnv`. This runs each environment in a separate process. Time the difference with 4 environments and 1000 steps on your machine.
3. Print `obs.dtype` for the vectorised BurgerBot observation. Is it `int64`? If you have a `Box` environment with `float32` observations, what dtype would you expect in the batch?

## Common Issues

**Problem:** `TypeError: make_env() missing required positional argument` when creating SyncVectorEnv
**Solution:** Make sure the factory returns a zero-argument callable. `make_env(i)` returns `_init` which takes no arguments — that is what `SyncVectorEnv` calls.
**Why:** `SyncVectorEnv` calls each factory with no arguments to create each environment.

**Problem:** `obs.shape` is `(4,)` instead of `(4, 4)` for CartPole
**Solution:** Make sure your environments are Gymnasium-compliant and return NumPy arrays from `reset()` and `step()`. A plain integer observation will be stacked as a 1D array.
**Why:** `SyncVectorEnv` stacks individual observations along a new batch dimension. If the observation is a scalar (shape `()`), the batch has shape `(n_envs,)`. If it is `(4,)`, the batch has shape `(n_envs, 4)`.
