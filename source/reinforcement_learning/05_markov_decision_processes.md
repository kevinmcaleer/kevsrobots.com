---
title: Markov Decision Processes
description: Understand why your robot only needs to know where it is right now — and the mathematical idea that makes that claim safe.
layout: lesson
type: page
cover: assets/cover.jpg
date_updated: 2026-06-13
---

![Cover](assets/cover.jpg){:class="cover"}

---

You might have noticed something quietly assumed in the last few lessons: when the agent chooses an action, it only looks at the *current* state — not at the history of states that led here. This seems almost too simple. Surely the robot should remember where it came from?

This lesson explains why that simplification is not just convenient but theoretically justified — and what it means in practice for our robot.

---

## The Markov Property

The core idea is called the **Markov property**, named after the Russian mathematician Andrey Markov. In plain English:

> **The future depends only on the present, not on the past.**

More precisely: once you know the current state, knowing the entire history of previous states gives you no additional information about what will happen next.

For a robot, this translates to: "where I am and what my sensors read right now is all I need to decide what to do next." I don't need to remember that I came from the top-left corner via a clockwise spiral three seconds ago. That history is already encoded in my current position and sensor reading.

---

## When Is This Actually True?

The Markov property holds cleanly when the state captures everything relevant to the future. For our discretised grid world:

```python
# State = (row, col, heading, distance_band)
# This captures:
#   - WHERE the robot is right now (row, col)
#   - WHICH WAY it's pointing (heading)
#   - WHAT it can see directly ahead (distance_band)
#
# Given this information, the consequences of any action
# (forward, turn_left, turn_right) are fully determined.
# The history of how we got here doesn't change those consequences.

state = (2, 3, "east", "far")  # robot at (2,3), facing east, clear ahead

# Whether the robot got here by turning from the north
# or by driving straight from the west doesn't matter.
# The correct action is the same either way.
```

In practice, the Markov property is approximately true for our robot. A sensor reading plus a grid position usually captures enough to make a good decision. The approximation breaks down in edge cases (e.g. if the robot is slipping on a smooth floor, its physical speed might matter too), but for a learning exercise with discrete states, it's an excellent model.

---

## The Full MDP Framework

A **Markov Decision Process (MDP)** is the mathematical framework that formalises everything we've discussed so far. It has four components:

| Component | Symbol | Our robot's version |
|---|---|---|
| Set of states | S | All (row, col, heading, distance_band) combinations |
| Set of actions | A | {forward, turn_left, turn_right, stop} |
| Transition function | T(s, a, s') | Which state follows when you take action a in state s |
| Reward function | R(s, a, s') | The reward received for that transition |
{:class="table table-single"}

In a known environment, T and R are given as tables. In **model-free reinforcement learning** (which is what Q-learning is), the agent doesn't know T or R in advance — it discovers them through experience.

```python
# An MDP in miniature: a corridor with three cells

# States: positions 0, 1, 2 (left to right)
# Actions: "left", "right"
# Goal: reach position 2

S = [0, 1, 2]
A = ["left", "right"]

# Transition table: T[state][action] = next_state
T = {
    0: {"left": 0,  "right": 1},   # at left wall; can't go further left
    1: {"left": 0,  "right": 2},
    2: {"left": 1,  "right": 2},   # at goal; can't go further right
}

# Reward table: R[state][action] = reward
R = {
    0: {"left": -0.1, "right": -0.1},
    1: {"left": -0.1, "right": 10.0},   # reaching goal from state 1 is great!
    2: {"left": -0.1, "right": -0.1},
}
```

This tiny MDP is fully specified. An agent can solve it by inspection: always go right until you hit state 2. For a 5×5 grid with obstacles, the correct policy isn't obvious by inspection — that's exactly where Q-learning shines.

---

## Episodes and Terminal States

An MDP run is divided into **episodes**: sequences of state-action-reward triples that start at an initial state and end at a **terminal state** (the goal, or a dead end like a crash).

```python
# A simple episode trace
episode = [
    # (state,         action,      reward)
    ((1, 0, "east", "far"),  "forward",     +1.0),
    ((1, 1, "east", "far"),  "forward",     +1.0),
    ((1, 2, "east", "near"), "turn_left",   -0.1),
    ((1, 2, "north","far"),  "forward",     +1.0),
    ((0, 2, "north","far"),  "forward",    +50.0),  # reached goal!
]
# Episode ends. Reset to start for the next episode.
```

Between episodes, the environment resets (robot back to starting position, obstacles in their original places). The agent keeps its Q-table — the knowledge accumulated across all previous episodes is preserved.

---

## Try It Yourself

1. Sketch the tiny corridor MDP above on paper. Trace through three episodes starting from state 0 with a random policy (randomly choose "left" or "right" each step). Record the reward each step. Notice how rarely you stumble upon the +10 reward by pure chance.

2. Extend the corridor MDP to five cells with the goal at cell 4. Add a "death trap" at cell 2 (reward -50, episode ends). Write out the state space S, and action-transition table T.

3. For the BurgerBot grid world on a 4×4 grid with 4 headings and 3 distance bands: calculate the total number of states |S| and the total number of state-action pairs |S| × |A|. This is the maximum number of entries in your Q-table.

---

## Common Issues

**"Does the Markov property mean I can never use memory?"**
Not exactly. In a Markov-violating environment (partially observable, or where history genuinely matters), you can make the state Markov by including the relevant history *in* the state. For example, if the robot's speed matters, include a discretised speed in the state. The cost is a larger state space.

**"What if my transitions are random? The robot might slip and end up somewhere different."**
MDPs handle probabilistic transitions fine — T(s, a, s') is technically a probability distribution over next states, not a deterministic mapping. Q-learning still works because it averages over many experiences of the same (s, a) pair. We'll see this implicitly in training.

**"Do I need to code the full MDP explicitly before training?"**
Not with Q-learning! That's the beauty of model-free RL. The agent learns Q-values directly from experience without ever constructing T or R. We'll build the grid-world environment in lesson 10 and the agent will infer everything it needs through interaction.

---

Next up: discount factors and the Bellman idea — why a reward tomorrow is worth slightly less than the same reward today.

---
