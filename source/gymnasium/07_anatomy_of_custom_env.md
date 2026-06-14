---
title: Anatomy of a Custom Environment
description: Learn the gym.Env contract — what you must implement to write a valid Gymnasium environment — and validate it with check_env.
layout: lesson
type: page
cover: assets/cover.jpg
date_updated: 2026-06-14
---

![Cover](assets/cover.jpg){:class="cover"}

---

## The Contract

To write a Gymnasium environment, you subclass `gym.Env` and implement four things:

1. `observation_space` and `action_space` — defined in `__init__`
2. `reset(*, seed=None, options=None)` — returns `(obs, info)`
3. `step(action)` — returns `(obs, reward, terminated, truncated, info)`
4. Optionally: `render()` and `metadata`

That is the entire contract. Anything that satisfies these four points will work with Stable-Baselines3, Gymnasium wrappers, and any other tool that speaks the API.

## A Minimal Example: The Corridor

Let's build a minimal environment before we tackle BurgerBot. A one-dimensional corridor: the agent starts at position 0 and must reach position 9. It can move left or right. Hitting either wall (position -1 or 10) ends the episode.

This is intentionally tiny — the goal is to see every piece of the contract clearly, not to solve an interesting problem.

```python
import numpy as np
import gymnasium as gym
from gymnasium import spaces

class CorridorEnv(gym.Env):
    """
    A 10-cell 1D corridor. Agent starts at 0, goal is 9.
    Actions: 0=left, 1=right.
    Observation: current position (integer 0-9).
    """

    # metadata tells wrappers what render modes this env supports.
    metadata = {"render_modes": ["ansi"]}

    def __init__(self, render_mode=None):
        super().__init__()

        # The spaces MUST be defined in __init__.
        # Discrete(10) means the observation is an integer in {0, ..., 9}.
        self.observation_space = spaces.Discrete(10)
        # Discrete(2) means the agent can do action 0 (left) or 1 (right).
        self.action_space = spaces.Discrete(2)

        self.render_mode = render_mode
        self.position = 0    # current agent position

    def reset(self, *, seed=None, options=None):
        # Call super().reset(seed=seed) to let Gymnasium handle the RNG.
        # This gives you self.np_random for reproducible environment randomness.
        super().reset(seed=seed)

        self.position = 0    # always start at the left
        observation = self.position

        # reset() MUST return (observation, info).
        # info is a dict — empty is fine.
        return observation, {}

    def step(self, action):
        # Apply the action
        if action == 0:   # move left
            self.position -= 1
        elif action == 1: # move right
            self.position += 1

        # Determine outcome
        hit_wall = self.position < 0 or self.position >= 10
        reached_goal = self.position == 9

        # Compute reward
        if hit_wall:
            reward = -5.0
            self.position = max(0, min(9, self.position))  # clamp position
        elif reached_goal:
            reward = 10.0
        else:
            reward = -0.1   # small cost per step to encourage speed

        # terminated: goal reached or wall hit
        terminated = reached_goal or hit_wall
        # truncated: never — this env has no time limit of its own
        truncated = False

        observation = self.position

        # step() MUST return (observation, reward, terminated, truncated, info).
        return observation, reward, terminated, truncated, {}

    def render(self):
        if self.render_mode == "ansi":
            # Build a simple text representation
            cells = ["."] * 10
            cells[self.position] = "A"   # agent
            cells[9] = "G"               # goal
            return "[" + " ".join(cells) + "]"
```

## Validating with check_env

Gymnasium ships a checker that verifies your environment follows the contract. It catches common mistakes: wrong return types, out-of-range observations, incorrect space definitions.

```python
from gymnasium.utils.env_checker import check_env

env = CorridorEnv()
check_env(env)
print("CorridorEnv passed check_env!")
```

If everything is correct, `check_env` prints nothing and returns. If there is a problem, it raises an error with a clear description of what went wrong.

Run `check_env` on every custom environment you write. It catches:
- Missing `super().reset(seed=seed)` calls
- Observations returned as plain Python `int` or `float` instead of the correct type
- Observations outside the declared space bounds
- Missing or wrong keys in returned info dicts (Gymnasium 1.x is lenient here but warns)

## Running the Corridor

```python
env = CorridorEnv(render_mode="ansi")
obs, info = env.reset(seed=0)

print(env.render())   # [A . . . . . . . . G]

done = False
total_reward = 0
steps = 0

while not done:
    action = 1   # always move right
    obs, reward, terminated, truncated, info = env.step(action)
    total_reward += reward
    steps += 1
    done = terminated or truncated

print(env.render())   # [. . . . . . . . . G] — agent is at goal
print(f"Steps: {steps}, Total reward: {total_reward:.1f}")
# Steps: 9, Total reward: 9.1 (9 steps at -0.1 + 10.0 for goal)
env.close()
```

## Common Mistakes and What check_env Catches

### Forgetting super().reset(seed=seed)

```python
def reset(self, *, seed=None, options=None):
    # BAD: no super() call
    self.position = 0
    return self.position, {}
```

`check_env` will warn that `self.np_random` is not properly initialised. Always call `super().reset(seed=seed)`.

### Returning the Wrong Type

```python
def reset(self, *, seed=None, options=None):
    super().reset(seed=seed)
    self.position = 0
    return self.position, {}   # int, not np.int64
```

`Discrete(10)` expects a NumPy scalar or integer compatible with its dtype. `check_env` may warn about this. Safe fix:

```python
return np.int64(self.position), {}
```

### Off-by-One in Space Definition

```python
# BAD: positions are 0-9 but space says only 9 values
self.observation_space = spaces.Discrete(9)
# position=9 (the goal) is outside this space!
```

`check_env` will catch this the first time the agent reaches the goal. The space must cover every possible observation value including terminal ones.

## Try It Yourself

1. Add a `max_steps=20` limit to `CorridorEnv`. Set `truncated = (self.steps >= self.max_steps)` and test with `check_env`. Does the corridor still solve with `action=1` always?
2. Change the reward function so the agent gets a bonus proportional to its position (`reward += self.position * 0.1`). Does this help or hurt a random agent?
3. Write a `SnailEnv` where the observation is just a timestep counter (0 to 9) and the only action is "wait". The episode ends after 10 steps. Run `check_env`. This is the simplest possible environment — it is a useful debugging scaffold.
