---
title: Seeding and the Episode Lifecycle
description: Make your training runs reproducible with env.reset(seed=...) and action_space.seed(), then learn the full episode lifecycle and env.close().
layout: lesson
type: page
cover: assets/cover.jpg
date_updated: 2026-06-14
---

![Cover](assets/cover.jpg){:class="cover"}

---

## Why Reproducibility Matters

Two common problems in RL projects:

1. "It worked great yesterday but now it fails." The training was random. You cannot reproduce it.
2. "My agent beats the baseline but my friend's identical code doesn't." Different random seeds led to different outcomes.

Reproducibility is not a luxury. It is how you know whether a change in your code actually improved the agent, or whether you just got lucky with a different random trajectory.

## Seeds in Gymnasium

Gymnasium's randomness lives in two places: the environment itself and the action space sampler. Seeding both gives fully reproducible episodes.

```python
import gymnasium as gym
import numpy as np

env = gym.make("CartPole-v1")

# Seed the environment. This affects the starting state and any
# environment-internal randomness.
obs, info = env.reset(seed=42)

# Seed the action space. This affects env.action_space.sample().
env.action_space.seed(42)

# Now a run with these two seeds will produce the exact same
# sequence of observations for the same sequence of actions.
action = env.action_space.sample()
print("First action:", action)   # always the same with seed=42

env.close()
```

> **Note:** Seeding the environment does not seed NumPy or Python's `random` module. If your training loop uses `random.random()` or `np.random.rand()`, seed those separately:
> ```python
> import random, numpy as np
> random.seed(42)
> np.random.seed(42)
> ```

## Seed Once Per Run, Not Per Episode

A common mistake is re-seeding at the start of every episode:

```python
# BAD: re-seeds every episode, which resets the RNG to the same state.
# Every episode becomes identical.
for episode in range(100):
    obs, info = env.reset(seed=42)   # same seed every time!
```

Instead, seed once before the loop begins, then call `env.reset()` without a seed for subsequent episodes:

```python
# GOOD: seed once, then let the RNG advance naturally.
obs, info = env.reset(seed=42)

for episode in range(100):
    # ... run episode ...
    obs, info = env.reset()   # no seed — RNG advances from where it left off
```

If you want different but reproducible behaviour per experiment, change the seed number between full runs:

```python
for trial_seed in [0, 1, 2, 3, 4]:
    env = gym.make("CartPole-v1")
    obs, info = env.reset(seed=trial_seed)
    env.action_space.seed(trial_seed)
    # ... full training run ...
    env.close()
```

## The Full Episode Lifecycle

Let's put the complete lifecycle in one place. This is the pattern every Gymnasium training loop should follow:

```python
import gymnasium as gym

env = gym.make("CartPole-v1")

# 1. Seed and get the first observation.
obs, info = env.reset(seed=0)
env.action_space.seed(0)

for episode in range(10):

    done = False
    episode_reward = 0

    # 2. Run steps until the episode ends.
    while not done:
        # 3. Choose an action.
        action = env.action_space.sample()

        # 4. Apply the action and get feedback.
        obs, reward, terminated, truncated, info = env.step(action)
        episode_reward += reward

        # 5. Check for episode end.
        done = terminated or truncated

    print(f"Episode {episode + 1}: reward = {episode_reward:.1f}")

    # 6. Reset for the next episode (no seed after the first).
    obs, info = env.reset()

# 7. Always close the environment when done.
env.close()
```

Each numbered step corresponds to a specific phase:

| Phase | What happens |
|-------|-------------|
| 1. Seed + reset | Environment initialises to a start state; observation returned |
| 2. Episode loop | Run until `done` |
| 3. Choose action | Agent or random sampler picks an action |
| 4. Step | Environment applies action, returns 5 values |
| 5. Check done | `terminated or truncated` ends the episode |
| 6. Reset | New episode begins, RNG advances |
| 7. Close | Releases rendering resources and file handles |

## Why env.close() Matters

`env.close()` releases resources: the rendering window (if open), any file handles the environment opened, and any background threads it started. For environments with `render_mode="human"`, forgetting `env.close()` can leave zombie processes.

Using a context manager avoids forgetting:

```python
import gymnasium as gym

with gym.make("CartPole-v1") as env:
    obs, info = env.reset(seed=0)
    obs, reward, terminated, truncated, info = env.step(0)
    print(reward)
# env.close() is called automatically when the with block exits
```

This pattern is especially useful in scripts where an exception might interrupt the loop before `env.close()` is reached.

## Comparing Two Seeds

Here is a quick script that shows how seed choice affects a run:

```python
import gymnasium as gym

for seed in [0, 1, 2]:
    env = gym.make("CartPole-v1")
    obs, info = env.reset(seed=seed)
    env.action_space.seed(seed)

    rewards = []
    for episode in range(5):
        done = False; ep_r = 0
        while not done:
            action = env.action_space.sample()
            obs, r, terminated, truncated, info = env.step(action)
            ep_r += r
            done = terminated or truncated
        rewards.append(ep_r)
        obs, info = env.reset()

    env.close()
    print(f"Seed {seed}: episode rewards = {[int(r) for r in rewards]}")
```

You should see different episode reward sequences for each seed, but the same sequence every time you run the script with the same seed.

## Try It Yourself

1. Run the comparison script above three times without changing the code. Are the results identical each run? They should be, because the seeds are fixed.
2. Remove the `env.action_space.seed(seed)` line and run again. Are the results still reproducible? Why or why not?
3. Write a function `run_experiment(seed, num_episodes)` that returns the list of episode rewards for a given seed. Use it to compare seeds 0 through 9 and find which seed produces the highest average reward with a random agent on CartPole.

## Common Issues

**Problem:** Training results differ between runs even with the same seed.
**Solution:** Check that you are also seeding `random` and `numpy.random` if your training code uses them, and that you are not multi-threading (threads introduce non-determinism).
**Why:** Gymnasium's internal RNG is seeded, but if your agent code uses Python's `random` module or NumPy separately, those have their own independent RNGs.

**Problem:** `env.reset(seed=42)` raises `TypeError: reset() got an unexpected keyword argument 'seed'`
**Solution:** You are using an old environment that has not been updated to the Gymnasium 1.x API. Update the package or find a newer version of the environment.
**Why:** Old OpenAI Gym environments had `env.seed(42)` as a separate call. The `seed=` parameter in `reset()` is a Gymnasium 1.x feature.
