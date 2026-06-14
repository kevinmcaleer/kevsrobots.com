---
layout: lesson
title: Wrapping the BurgerBot
author: Kevin McAleer
type: page
cover: assets/cover.jpg
date: 2026-06-14
previous: 07_anatomy_of_custom_env.html
next: 09_registering_and_rendering.html
description: Port the BurgerBot GridWorld from the previous course into a proper Gymnasium
  environment, validate it with check_env, and run a random-agent rollout.
percent: 54
duration: 8
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

## What We Are Porting

In the Reinforcement Learning for Beginners course, lesson 10, you built a `GridWorld` class that simulated the BurgerBot navigating a 7x7 grid. The grid looked like this:

```
. . . O . . .
. O . . . O .
. . . . . . .
O . . O . . .
. . . . . O .
. O . . . . .
. . . . . . G
```

The robot started at `(0, 0)` facing east and had to reach the goal at `(6, 6)` without hitting any of the 7 obstacles marked `O`. Its state at every step was a four-element tuple: `(row, col, heading, sensor)`. The sensor read the number of clear cells directly ahead, binned into three levels: near (0), medium (1), or far (2).

The GridWorld had three methods — `reset()`, `step()`, and (optionally) a display helper — and returned a three-tuple `(state, reward, done)`.

Now we are going to wrap that same world as a proper Gymnasium environment. The physics, the obstacles, the sensor, and the reward function stay identical. What changes is the interface.

## Design Decisions

Before writing the code, let's think through the design explicitly.

### Observation space

The state tuple has four components:

- `row`: 0 to 6, so 7 possible values
- `col`: 0 to 6, so 7 possible values
- `heading`: one of 4 directions (north, east, south, west), so 4 values
- `sensor`: near, medium, or far, so 3 values

`MultiDiscrete([7, 7, 4, 3])` describes this exactly. The total state space is `7 * 7 * 4 * 3 = 588` possible observations — small enough for a Q-table, which is why the previous course's tabular approach worked.

### Action space

Four actions — forward, turn_left, turn_right, stop — map naturally to `Discrete(4)`.

### terminated vs truncated

Looking at the original reward function:
- Hit an obstacle: reward `-10.0` and episode ends. This is a genuine terminal state: the robot has crashed. **`terminated = True`**
- Reached the goal: reward `+50.0` and episode ends. Also a genuine terminal state. **`terminated = True`**
- Hit the step limit: episode ends, no penalty. This is a time limit, not a failure. **`truncated = True`**

### The sensor

The sensor reads the number of clear cells directly ahead (up to 5) before hitting an obstacle or the grid boundary. It returns:
- 0 (near) if there are 0 or 1 clear cells
- 1 (medium) if there are 2 or 3 clear cells
- 2 (far) if there are 4 or more clear cells

This matches the `NEAR = 1`, `MEDIUM = 3` thresholds from the original GridWorld.

## The Complete BurgerBotEnv

```python
import numpy as np
import gymnasium as gym
from gymnasium import spaces

# ── World constants — identical to the RL course's GridWorld ─────────────────
GRID_ROWS, GRID_COLS = 7, 7
OBSTACLES = {(0,3),(1,1),(1,5),(3,0),(3,3),(4,5),(5,1)}
START, GOAL = (0,0), (6,6)
ACTIONS   = ["forward","turn_left","turn_right","stop"]
HEADINGS  = ["north","east","south","west"]
HEADING_DELTA = {"north":(-1,0),"east":(0,1),"south":(1,0),"west":(0,-1)}
NEAR, MEDIUM = 1, 3

class BurgerBotEnv(gym.Env):
    """
    The BurgerBot 7x7 grid-world as a Gymnasium environment.

    Observation: np.array([row, col, heading_index, sensor_level], dtype=int64)
        heading_index: 0=north, 1=east, 2=south, 3=west
        sensor_level:  0=near (<=1 clear cell ahead),
                       1=medium (2-3 clear cells),
                       2=far (>=4 clear cells)

    Actions: 0=forward, 1=turn_left, 2=turn_right, 3=stop

    Rewards:
        Hit obstacle:  -10.0  (terminated=True)
        Reach goal:    +50.0  (terminated=True)
        Move forward:   +1.0
        Turn or stop:   -0.1
        Time limit:      0.0  (truncated=True)
    """

    metadata = {"render_modes": ["ansi"]}

    def __init__(self, max_steps=150, render_mode=None):
        super().__init__()
        self.max_steps = max_steps
        self.render_mode = render_mode

        # MultiDiscrete([7, 7, 4, 3]) matches (row, col, heading, sensor)
        self.observation_space = spaces.MultiDiscrete([GRID_ROWS, GRID_COLS, 4, 3])
        self.action_space = spaces.Discrete(4)

    # ── Private helpers ───────────────────────────────────────────────────────

    def _sensor(self):
        """Count clear cells directly ahead, capped at 5. Return binned level."""
        dr, dc = HEADING_DELTA[self.heading]
        r, c = self.row + dr, self.col + dc
        n = 0
        while 0 <= r < GRID_ROWS and 0 <= c < GRID_COLS:
            if (r, c) in OBSTACLES:
                break
            n += 1
            if n >= 5:
                break
            r += dr; c += dc
        if n <= NEAR:   return 0   # near
        if n <= MEDIUM: return 1   # medium
        return 2                   # far

    def _obs(self):
        """Return the current observation as a numpy array."""
        return np.array(
            [self.row, self.col, HEADINGS.index(self.heading), self._sensor()],
            dtype=np.int64
        )

    # ── Gymnasium contract ────────────────────────────────────────────────────

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.row, self.col = START
        self.heading = "east"
        self.steps = 0
        return self._obs(), {}

    def step(self, action):
        self.steps += 1
        a = ACTIONS[action]
        hit = False
        moved = False

        if a == "forward":
            dr, dc = HEADING_DELTA[self.heading]
            nr, nc = self.row + dr, self.col + dc
            if (0 <= nr < GRID_ROWS and 0 <= nc < GRID_COLS
                    and (nr, nc) not in OBSTACLES):
                self.row, self.col = nr, nc
                moved = True
            else:
                hit = True
        elif a == "turn_left":
            self.heading = HEADINGS[(HEADINGS.index(self.heading) - 1) % 4]
        elif a == "turn_right":
            self.heading = HEADINGS[(HEADINGS.index(self.heading) + 1) % 4]
        # "stop" does nothing

        reached = (self.row, self.col) == GOAL

        if hit:
            reward = -10.0
        elif reached:
            reward = 50.0
        elif moved:
            reward = 1.0
        else:
            reward = -0.1

        terminated = reached or hit
        truncated  = self.steps >= self.max_steps

        return self._obs(), reward, terminated, truncated, {}

    def render(self):
        if self.render_mode != "ansi":
            return
        heading_char = {"north":"^","east":">","south":"v","west":"<"}
        out = ""
        for r in range(GRID_ROWS):
            for c in range(GRID_COLS):
                if (r, c) == (self.row, self.col):
                    out += heading_char[self.heading] + " "
                elif (r, c) in OBSTACLES:
                    out += "O "
                elif (r, c) == GOAL:
                    out += "G "
                else:
                    out += ". "
            out += "\n"
        return out
```

## Validating with check_env

Always run `check_env` before you train on a custom environment:

```python
from gymnasium.utils.env_checker import check_env

env = BurgerBotEnv()
check_env(env)
print("BurgerBotEnv passed check_env!")
env.close()
```

If this prints the success message without raising an exception, the environment is Gymnasium-compliant. If it raises, read the error — it will tell you exactly which part of the contract is violated.

## A Random-Agent Rollout

Let's watch the agent do something before we train it:

```python
env = BurgerBotEnv(render_mode="ansi")
obs, info = env.reset(seed=7)
print("Initial state:")
print(env.render())

total_reward = 0
done = False

for step in range(30):
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
    total_reward += reward
    done = terminated or truncated

    if done:
        print(f"Episode ended at step {step + 1}.")
        print(f"Terminated: {terminated}, Truncated: {truncated}")
        break

print(f"Total reward from random agent: {total_reward:.1f}")
print("\nFinal grid state:")
print(env.render())
env.close()
```

A random agent on BurgerBot usually hits an obstacle within the first 20 steps. That is fine — it shows the environment is working and the penalty is being applied.

## Comparing to the Original GridWorld

Here is a side-by-side of the key differences between the hand-rolled GridWorld and BurgerBotEnv:

| Aspect | Original GridWorld | BurgerBotEnv |
|--------|-------------------|--------------|
| Return from reset | `state` tuple | `(np.array, {})` |
| Return from step | `(state, reward, done)` | `(np.array, reward, terminated, truncated, {})` |
| State representation | Python tuple | NumPy int64 array |
| Episode end signal | Single `done` bool | Two separate flags |
| Space declaration | None (implicit) | `observation_space`, `action_space` |
| Interchangeable? | No | Yes — works with any Gymnasium-compatible library |
{:class="table table-single"}

The underlying physics — the grid, the obstacles, the sensor, the rewards — are exactly the same. Only the interface changed.

## Try It Yourself

1. Run `check_env(BurgerBotEnv())` and then deliberately break the environment by returning a Python `int` from `_obs()` instead of a NumPy array. What error does `check_env` produce?
2. Change `max_steps` to 10 and run the rollout. How often does the episode end due to truncation vs termination with a random agent?
3. Add a method `state_count(num_episodes=100)` that runs `num_episodes` random-agent episodes and counts how many unique observations were seen. How does this compare to the theoretical maximum of 588?

## Common Issues

**Problem:** `check_env` warns "The observation returned by the `reset()` method is not within the observation space"
**Solution:** Make sure `_obs()` returns `dtype=np.int64`. Some systems default NumPy to `int32`.
**Why:** `MultiDiscrete` declares `int64` dtype. NumPy's `np.array([...])` defaults to `int64` on most platforms but `int32` on some (especially 32-bit Python builds).

**Problem:** The sensor always returns 0 (near) even when the path is clear.
**Solution:** Check the HEADING_DELTA dictionary. "north" should be `(-1, 0)` (row decreases going north). If row 0 is the top of the grid, this is correct.
**Why:** In grid coordinates with row 0 at the top, moving north decreases the row index.
