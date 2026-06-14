---
layout: lesson
title: Terminated vs Truncated
author: Kevin McAleer
type: page
cover: assets/cover.jpg
date: 2026-06-14
previous: 02_first_environment.html
next: 04_spaces.html
description: Learn why Gymnasium separates the done flag into terminated and truncated,
  and why this distinction matters for correct RL training.
percent: 24
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

## The Old Way: One Flag

In the previous course, the BurgerBot simulator returned a single `done` flag:

```python
# The hand-built step() from the RL course
state, reward, done = world.step(action)
```

`done` was `True` in two situations:
- The robot hit an obstacle (bad ending — episode is truly over)
- The episode hit `MAX_STEPS = 150` (time limit — we just stopped the episode)

Those two situations are logically different, and collapsing them into one flag causes a subtle but real training problem. Let's look at why.

## The Bootstrap Problem

Q-learning updates a value estimate based on what the agent expects to earn from the next state onward:

```python
# The update from the RL course
# td_target = reward + gamma * max(q_table[next_state])
# q_table[state][action] += alpha * (td_target - q_table[state][action])
```

The term `gamma * max(q_table[next_state])` is called the bootstrap: we are using our current estimate of the next state's value to update our estimate of the current state's value.

When the episode genuinely ends — the pole falls, the robot crashes, the goal is reached — there is no next state. The bootstrap term should be zero. That is correct.

When the episode ends only because of a time limit, the agent is actually in a perfectly valid state. The environment could continue from there. The bootstrap term should NOT be zero — it should reflect how much future reward the agent could still earn.

If you treat a time-limit ending the same as a genuine terminal state, you are incorrectly telling the agent "there is no future value from this state" when there actually is. For small, fast-converging problems this error is often small enough to not matter. For harder problems — or when you are using the time limit to learn about regions of the state space the agent hasn't reached yet — it introduces a real bias.

## The New Way: Two Flags

Gymnasium 1.x separates the concept into two flags:

```python
obs, reward, terminated, truncated, info = env.step(action)
```

- `terminated` is `True` when the episode ends for a reason defined by the environment: the goal was reached, the agent died, the pole fell past the threshold, the robot hit a wall. This is a genuine terminal state. Bootstrap should be zero.
- `truncated` is `True` when the episode ends because of an external limit: a time limit, a step budget, or any stopping condition imposed from outside the environment's own rules. This is NOT a terminal state. Bootstrap should continue.

The episode is over (from the training loop's perspective) if either is `True`. But a well-written RL algorithm can handle them differently.

Here is what the distinction looks like in practice:

```python
import gymnasium as gym

env = gym.make("CartPole-v1")
obs, info = env.reset(seed=0)

for step in range(1000):
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)

    if terminated:
        # Pole fell — genuine terminal state.
        # Bootstrap = 0. No future reward possible.
        print(f"Step {step}: TERMINATED (pole fell). Resetting.")
        obs, info = env.reset()

    elif truncated:
        # Time limit hit — agent is still in a valid state.
        # Bootstrap could continue. Policy was cut short, not beaten.
        print(f"Step {step}: TRUNCATED (time limit). Resetting.")
        obs, info = env.reset()

env.close()
```

## What This Means for BurgerBot

Look back at the BurgerBot simulator from the previous course. Its `done` flag was set `True` in three cases:

1. The robot hit an obstacle (hit = True) — **this is `terminated`**
2. The robot reached the goal — **this is also `terminated`**
3. `steps >= MAX_STEPS` — **this is `truncated`**

The validated BurgerBotEnv you will build in lesson 8 handles all three correctly:

```python
# From BurgerBotEnv.step()
reached = (self.row, self.col) == GOAL

if hit:
    reward = -10.0
elif reached:
    reward = 50.0
elif moved:
    reward = 1.0
else:
    reward = -0.1

terminated = reached or hit          # genuine endings
truncated = self.steps >= self.max_steps   # time limit only

return self._obs(), reward, terminated, truncated, {}
```

A hit gives `terminated=True, truncated=False`. A time limit gives `terminated=False, truncated=True`. Both end the episode from the training loop's point of view, but Stable-Baselines3 (and any correct RL implementation) handles them differently.

## The Simple Training Loop Pattern

For most hand-written training loops, you handle both flags the same way at the episode level:

```python
done = terminated or truncated
if done:
    obs, info = env.reset()
```

This is fine for tabular Q-learning where the distinction has little practical effect. When you move to SB3 in lessons 12 and 13, the library handles the distinction for you internally — which is one more reason to use a standard API.

## Try It Yourself

1. Run the code snippet above with `render_mode` omitted. After 200 steps, count how many `TERMINATED` events and how many `TRUNCATED` events you saw. CartPole's default time limit is 500 steps per episode — does this match your truncation count?
2. Change the random policy to `action = 0` (always push left). Does the pole fall faster, making `TERMINATED` more common?
3. Read the CartPole source code to find where `terminated` and `truncated` are set. You can find it by running `print(env.spec.entry_point)` — it will tell you the module path.

## Common Issues

**Problem:** Code written for old OpenAI Gym gives `ValueError: too many values to unpack`
**Solution:** Change `obs, reward, done, info = env.step(action)` to `obs, reward, terminated, truncated, info = env.step(action)`.
**Why:** Gymnasium 1.x returns five values from `step()`. The old Gym returned four.

**Problem:** `AttributeError: 'tuple' object has no attribute 'dtype'`
**Solution:** You are probably unpacking `reset()` as `obs = env.reset()` instead of `obs, info = env.reset()`.
**Why:** Gymnasium 1.x `reset()` returns a two-tuple. Assigning it to a single variable gives you the tuple, not the observation.
