---
layout: lesson
title: Gymnasium Wrappers
author: Kevin McAleer
type: page
cover: assets/cover.jpg
date: 2026-06-14
previous: 09_registering_and_rendering.html
next: 11_vectorized_envs.html
description: Learn how to use TimeLimit, RecordEpisodeStatistics, and custom wrappers
  to modify an environment's behaviour without editing its source code.
percent: 66
duration: 6
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

## What Is a Wrapper?

A wrapper is an environment that sits in front of another environment and intercepts some of its calls. The outer wrapper looks like a Gymnasium environment to the agent; the inner environment does not know it is being wrapped.

This is powerful because it lets you modify behaviour — clip rewards, add logging, reshape observations — without touching the original environment class. You can stack multiple wrappers and remove them later. If you later decide the reward clipping was a bad idea, you just remove that wrapper; the original environment is unchanged.

## The Three Wrappers You Will Use Most

### TimeLimit

Adds a step counter and sets `truncated=True` after a maximum number of steps. If you registered your environment with `max_episode_steps=150` in `gym.register`, Gymnasium wraps it in `TimeLimit` automatically.

You can apply it manually too:

```python
import gymnasium as gym
from gymnasium.wrappers import TimeLimit

from burgerbot_env import BurgerBotEnv   # or define it inline

base_env = BurgerBotEnv()               # no time limit
env = TimeLimit(base_env, max_episode_steps=50)   # now has a 50-step limit

obs, info = env.reset()
for _ in range(100):
    obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
    if terminated or truncated:
        print(f"truncated={truncated}, terminated={terminated}")
        break

env.close()
```

### RecordEpisodeStatistics

Adds episode length and cumulative reward to the `info` dict at the end of every episode. This is the standard way to track training progress without writing your own reward accumulator.

```python
from gymnasium.wrappers import RecordEpisodeStatistics

env = BurgerBotEnv()
env = RecordEpisodeStatistics(env)

obs, info = env.reset()
done = False

while not done:
    obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
    done = terminated or truncated

# At episode end, info contains:
# "episode": {"r": total_reward, "l": episode_length, "t": elapsed_time}
print("Episode info:", info["episode"])
env.close()
```

The `"r"` (return), `"l"` (length), and `"t"` (wall-clock time) fields are added by the wrapper. Stable-Baselines3 reads these automatically to produce training logs.

## Writing a Custom Wrapper

Gymnasium provides base classes for the most common customisation points:

- `ObservationWrapper` — override `observation(obs)` to transform observations
- `RewardWrapper` — override `reward(reward)` to transform rewards
- `ActionWrapper` — override `action(action)` to transform actions
- `Wrapper` — override `step()`, `reset()`, or both for full control

### A Reward Shaping Wrapper for BurgerBot

The raw BurgerBot rewards are: +50 for goal, -10 for crash, +1 for moving, -0.1 for turning. Let's add a shaping term that gives a small bonus for reducing the Manhattan distance to the goal. This is called reward shaping — giving the agent a denser signal to help it learn faster.

```python
import numpy as np
import gymnasium as gym

class ManhattanShapingWrapper(gym.RewardWrapper):
    """
    Adds a shaping bonus to BurgerBot's reward based on progress
    toward the goal. The bonus is the reduction in Manhattan distance
    to the goal this step.

    This gives the agent a denser reward signal without changing the
    fundamentals of the task.
    """

    GOAL_ROW, GOAL_COL = 6, 6

    def __init__(self, env):
        super().__init__(env)
        # We need access to the previous distance, which means we need to
        # track the agent's position across steps.
        self._prev_distance = None

    def reset(self, **kwargs):
        obs, info = self.env.reset(**kwargs)
        row, col = obs[0], obs[1]
        self._prev_distance = abs(row - self.GOAL_ROW) + abs(col - self.GOAL_COL)
        return obs, info

    def reward(self, reward):
        # Get current position from the unwrapped env
        row = self.env.row
        col = self.env.col
        current_distance = abs(row - self.GOAL_ROW) + abs(col - self.GOAL_COL)

        # Shaping bonus: positive if we got closer, negative if further away
        shaping = self._prev_distance - current_distance
        self._prev_distance = current_distance

        return reward + 0.5 * shaping   # scale shaping to be smaller than the real reward
```

Use it by wrapping:

```python
from burgerbot_env import BurgerBotEnv   # or define inline

base_env = BurgerBotEnv()
env = ManhattanShapingWrapper(base_env)

obs, info = env.reset(seed=0)
done = False
total = 0.0

while not done:
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
    total += reward
    done = terminated or truncated

print(f"Shaped total reward: {total:.2f}")
env.close()
```

### A Flattened Observation Wrapper

Stable-Baselines3's MlpPolicy expects a flat 1D observation array. BurgerBot's `MultiDiscrete` observation is already a 1D array of 4 integers, so it works fine. But if you had a 2D grid observation, you would need to flatten it:

```python
class FlattenObsWrapper(gym.ObservationWrapper):
    """Flattens a 2D observation into a 1D array."""

    def __init__(self, env):
        super().__init__(env)
        # Compute the new flattened shape and declare a new observation space
        old_shape = env.observation_space.shape
        flat_size = 1
        for dim in old_shape:
            flat_size *= dim
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf,
            shape=(flat_size,), dtype=np.float32
        )

    def observation(self, obs):
        return obs.flatten().astype(np.float32)
```

## Stacking Wrappers

You can apply multiple wrappers in sequence. The agent sees the outermost wrapper; each wrapper passes calls inward:

```python
env = BurgerBotEnv()
env = ManhattanShapingWrapper(env)       # reshape rewards
env = RecordEpisodeStatistics(env)       # track episode stats
env = TimeLimit(env, max_episode_steps=100)  # enforce time limit

# The agent interacts with env as if it is a normal Gymnasium environment.
# Internally: TimeLimit -> RecordEpisodeStatistics -> ManhattanShapingWrapper -> BurgerBotEnv
```

To access the original environment through wrappers, use `env.unwrapped`:

```python
print(type(env.unwrapped))   # <class 'BurgerBotEnv'>
```

## Try It Yourself

1. Stack `RecordEpisodeStatistics` on top of BurgerBotEnv and run 10 random-agent episodes. Print the `info["episode"]` dict after each episode. What is the average episode length?
2. Write a `NoisyObservationWrapper` that randomly flips one of the four observation values with probability 0.1. Think about how this would affect a trained agent (conceptually — no need to train).
3. Use `env.unwrapped` to confirm that after stacking three wrappers, the `unwrapped` attribute still points to your original `BurgerBotEnv` instance.

## Common Issues

**Problem:** `AttributeError: 'TimeLimit' object has no attribute 'row'`
**Solution:** Access the unwrapped environment: `env.unwrapped.row`. Wrappers only forward the standard Gymnasium methods; custom attributes need `env.unwrapped`.
**Why:** Wrappers delegate `step`, `reset`, `observation_space`, and `action_space`, but they do not expose custom attributes from the inner class.

**Problem:** `RecordEpisodeStatistics` does not add `info["episode"]` during the episode.
**Solution:** This is expected — the `"episode"` key is only added to `info` at the step where `terminated or truncated` is True.
**Why:** The wrapper accumulates the reward and length during the episode and only writes the summary at the terminal step.
