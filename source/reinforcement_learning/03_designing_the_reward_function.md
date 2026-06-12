---
title: Designing the Reward Function
description: Craft rewards that teach the right behaviour — and discover the sneaky ways a poorly designed reward can backfire.
layout: lesson
type: page
cover: assets/cover.jpg
date_updated: 2026-06-13
---

![Cover](assets/cover.jpg){:class="cover"}

---

The reward function is the most powerful lever you have in reinforcement learning — and the most dangerous one. It's the entire channel through which you communicate your goals to the agent. Get it right and the robot learns exactly what you want. Get it subtly wrong and you end up with a robot that scores top marks by doing something completely useless.

This lesson is about designing rewards deliberately. We'll use our obstacle-avoiding BurgerBot as the running example.

---

## What Makes a Good Reward?

A reward signal should answer one question clearly: **was that a step toward the goal?**

Good rewards are:

- **Immediate** — the agent receives feedback close in time to the action that caused it
- **Proportional** — bigger positive or negative values for more significant outcomes
- **Specific** — tied to observable facts about the state, not to vague intentions
- **Sparse enough to avoid confusion, dense enough to provide signal** — a balance we'll discuss below

---

## Rewards for Obstacle Avoidance

Let's think through the task. Our BurgerBot should:

1. Move forward to explore the space
2. Avoid hitting obstacles
3. Ideally, reach a designated goal area

Here's a first attempt at a reward function:

```python
def compute_reward(next_state, hit_obstacle, reached_goal, moved_forward):
    """
    Returns a float reward value based on what just happened.

    next_state:     string label for the new sensor reading
    hit_obstacle:   bool — did the robot collide?
    reached_goal:   bool — did the robot reach the target cell?
    moved_forward:  bool — was the chosen action 'forward'?
    """
    if hit_obstacle:
        return -10.0    # strong penalty for collision

    if reached_goal:
        return +50.0    # large bonus for success

    if moved_forward:
        return +1.0     # small bonus: we want forward progress

    return -0.1         # tiny penalty for standing still or turning
                        # discourages aimless spinning
```

The small penalty for non-forward actions (-0.1) is subtle but important. Without it, a robot might learn that the safest strategy is to turn on the spot indefinitely — no collisions, no negative rewards. With it, doing nothing slowly accumulates cost, so the robot is nudged toward making progress.

---

## Reward Hacking: When Robots Find Loopholes

Here's where things get interesting. Agents are ruthlessly literal. They optimise for whatever you measure, not for what you *meant* to measure.

### The Statue Problem

Suppose you only penalise collisions. The agent quickly discovers the perfect strategy: **never move**. A stationary robot never crashes. Reward: zero collisions, maximum score.

```python
# Reward that seems fine but creates a statue
def naive_reward(hit_obstacle):
    if hit_obstacle:
        return -10.0
    return 0.0          # no penalty for standing still
                        # so the agent just... stands still
```

The robot isn't stupid. It genuinely found the optimal policy for *this* reward function. The problem is the reward function didn't capture what we actually wanted.

### The Spinner Problem

Suppose you reward forward progress (cells moved toward the goal) without penalising turning:

```python
# Slightly better, but still exploitable
def progress_only_reward(cells_toward_goal):
    return cells_toward_goal * 2.0  # only reward forward progress
```

A clever agent on a symmetric map might discover that spinning in a tight circle accumulates tiny amounts of "progress" on each half-rotation. Again, technically optimal for the measured reward.

### The Fix: Balance Multiple Signals

A robust reward function usually combines several signals with carefully chosen scales:

```python
def balanced_reward(hit_obstacle, reached_goal, moved_forward, steps_taken):
    """
    A more robust reward function for BurgerBot navigation.

    The scales matter: collision penalty >> goal bonus >> progress bonus
    >> idleness penalty. This ordering ensures the agent prioritises
    safety first, then success, then efficiency.
    """
    if hit_obstacle:
        return -10.0    # largest penalty: avoid crashes above all

    if reached_goal:
        return +50.0    # largest bonus: completing the task is the goal

    reward = 0.0

    if moved_forward:
        reward += 1.0   # small bonus per forward step

    # Tiny per-step penalty to discourage unnecessarily long routes.
    # Without this, the agent might reach the goal but take 200 steps
    # when 20 would do.
    reward -= 0.05

    return reward
```

The key insight: **the relative scale of rewards matters as much as their sign**. A collision penalty of -10 vs a goal bonus of +50 tells the agent that reaching the goal is worth enduring some risk, but not unlimited risk.

---

## Reward Shaping

Sometimes the goal is so far away that the agent never stumbles upon it by chance, so it never receives the large positive reward and can't learn anything useful. This is the **sparse reward problem**.

The solution is **reward shaping**: adding intermediate rewards that guide the agent toward the goal without completely specifying the solution.

```python
def shaped_reward(distance_to_goal, prev_distance_to_goal,
                  hit_obstacle, reached_goal):
    """
    Potential-based shaping: reward the agent for reducing distance
    to the goal, not just for arriving.

    This gives the agent a useful gradient to follow even when
    the goal is far away and rarely reached early in training.
    """
    if hit_obstacle:
        return -10.0

    if reached_goal:
        return +50.0

    # Progress toward goal (can be negative if moving away)
    progress = prev_distance_to_goal - distance_to_goal
    shaping_bonus = progress * 2.0

    return shaping_bonus - 0.05   # progress bonus minus small step cost
```

> **Warning:** Reward shaping is powerful but can introduce new reward-hacking opportunities. Always test your shaped reward in simulation and watch for unexpected behaviours before deploying.

---

## Reward Design Checklist

Before finalising a reward function, ask:

- Does the agent get positive reward for *doing something useful*, not just for *avoiding bad things*?
- Is there a per-step cost that prevents the agent from idling?
- Is the goal reward large enough that reaching it is always worth the effort?
- Could an agent score well without actually solving the task?
- Are the reward magnitudes in a sensible ratio to each other?

---

## Try It Yourself

1. Write a reward function for a different task: a robot arm that must pick up a block and place it in a bin. What events earn positive reward? What earns a penalty?

2. Take the naive "collision-only" reward function above and predict: what policy will an agent learn? Run it in your head for ten "episodes" where the agent acts randomly.

3. Modify `balanced_reward()` to add an extra penalty when the robot turns more than three times in a row without going forward. Why might this be useful? What risk does it introduce?

---

## Common Issues

**"My agent learned to stay still — it never moves at all."**
You're missing a per-step cost or a forward-progress bonus. Add a small negative reward for each time step and a small positive reward for the "forward" action.

**"My agent crashes immediately every episode and never improves."**
The collision penalty might be dominating before the agent has any chance to learn. Try temporarily reducing its magnitude, or make the goal closer to the start in early training.

**"The agent reaches the goal but takes forever to get there."**
Add a small per-step cost (e.g. -0.05 per step) to incentivise efficiency without making steps so costly that the agent avoids moving at all.

---

Next up: we'll look at how to translate the continuous physical world into the discrete states a Q-table can work with.

---
