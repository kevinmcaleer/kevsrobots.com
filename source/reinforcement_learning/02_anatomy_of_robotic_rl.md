---
title: The Anatomy of Robotic RL
description: Define the core reinforcement learning vocabulary — agent, environment, state, action, reward — in concrete robot terms.
layout: lesson
type: page
cover: assets/cover.jpg
date_updated: 2026-06-13
---

![Cover](assets/cover.jpg){:class="cover"}

---

Every field has its jargon, and reinforcement learning is no exception. The good news is that the core vocabulary is small — five terms cover almost everything — and each one maps neatly onto something physical on your robot.

Let's pin down each concept in terms of the BurgerBot-style robot we're going to train.

---

## The Five Core Concepts

### Agent

The **agent** is the decision-maker — the code that looks at the current situation and chooses what to do next.

For our robot, the agent is not the whole MicroPython program. It's specifically the part that takes in a sensor reading and returns a motor command. Everything else — the physics of wheels, the reflection of ultrasound off a wall — is outside the agent's control.

```python
# The agent is just this decision function:
def agent_choose_action(state):
    # Look up the best known action for this state
    return best_action_from_q_table(state)
```

### Environment

The **environment** is everything the agent is *not*. It includes:

- The physical arena (walls, obstacles, goal)
- The robot's chassis, motors, and wheels
- The laws of physics (friction, momentum)
- The sensor (ultrasonic distance readings)

The environment receives an action from the agent, updates its state, and hands back an **observation** and a **reward**. The agent never directly touches the environment's internals — it can only send actions and receive observations in return.

In our simulation, the environment is a Python object. On the real robot, the environment is the physical world.

### State

The **state** is a snapshot of the environment that the agent uses to make decisions.

For our robot, a state might be: "the ultrasonic sensor is reading 12 cm, and I'm currently moving forward." We need to express this as something the agent can index into a table, so we'll convert it into a simple label — more on that in lesson 4.

```python
# A state could be as simple as a string label
state = "near"          # obstacle closer than 15 cm
state = "medium"        # obstacle between 15 and 40 cm
state = "far"           # obstacle further than 40 cm
```

The agent doesn't know the whole environment — it only sees the state. This is an important constraint: the agent must make good decisions using only the information available in the state.

### Action

An **action** is a choice the agent makes. In our robot's case, the action is a motor command:

```python
ACTIONS = ["forward", "turn_left", "turn_right", "stop"]
```

The agent picks one action per time step. The environment executes it, physics happen, and a new state appears.

### Reward

The **reward** is a numerical signal that tells the agent whether the action it just took was a good idea.

- A **positive reward** ("well done, keep doing that") pushes the agent to take that action more often in similar states.
- A **negative reward** (a "penalty" or "cost") pushes it away from that action.

```python
# Simple reward function for obstacle avoidance
def compute_reward(new_state, hit_obstacle, reached_goal):
    if hit_obstacle:
        return -10.0    # strong discouragement
    if reached_goal:
        return +50.0    # strong encouragement
    if new_state == "far":
        return +1.0     # mild encouragement for open space
    return 0.0          # neutral
```

The reward is the only feedback the agent ever gets from the environment. Designing it carefully is one of the most important skills in applied RL — we'll dedicate the whole next lesson to it.

---

## The Agent-Environment Loop

These five concepts connect in a cycle that repeats many thousands of times during training:

```
  ┌──────────────────────────────────────────┐
  │                                          │
  │   AGENT                                  │
  │   ┌──────────────────┐                   │
  │   │                  │                   │
  │   │  observe state s │◄──────────────────┤
  │   │  choose action a │                   │
  │   │                  │                   │
  │   └────────┬─────────┘                   │
  │            │ action a                    │
  │            ▼                             │
  │   ENVIRONMENT                            │
  │   ┌──────────────────┐                   │
  │   │                  │                   │
  │   │  execute action  │                   │
  │   │  compute reward  ├──► reward r       │
  │   │  new state s'    ├──► new state s'  ─┘
  │   │                  │
  │   └──────────────────┘
```

In code, one step of this loop looks like:

```python
# One time step in the agent-environment loop
action = agent.choose(state)          # agent decides
next_state, reward, done = env.step(action)  # environment responds
agent.learn(state, action, reward, next_state)  # agent updates its knowledge
state = next_state                    # move forward in time
```

This loop runs inside an **episode** — a single attempt at the task, from start to finish (or until a time limit). After an episode ends (the robot reached the goal, or crashed, or ran out of time), the environment resets and a new episode begins.

---

## Mapping to Real Hardware

Here's how the abstract concepts map to the physical BurgerBot:

| RL concept | On the real robot |
|---|---|
| Agent | The MicroPython control script |
| Environment | The physical arena and floor |
| State | Discretised HC-SR04 reading (near / medium / far) |
| Action | Motor commands (forward, turn left, turn right, stop) |
| Reward | Computed after each move (collision = negative, progress = positive) |
| Episode | One power-on navigation run until goal or crash |
{:class="table table-single"}

In simulation, all of these exist as Python objects. The only difference on the real robot is that `env.step()` sends actual voltage to motors and reads actual ultrasound — the agent code stays almost identical.

---

## Try It Yourself

1. Write out the state-action-reward loop for a different robot task — say, a robot arm picking up a block. What would the states be? What are the possible actions? What would earn a reward?

2. For the obstacle-avoidance task, try to think of a state that provides *too little* information. What could go wrong if the agent only knew "obstacle detected: yes/no" (one bit of state) rather than the actual distance band?

3. For the same task, think of a state with *too much* information — say, the raw 16-bit distance reading from the sensor in millimetres. Why might having 65,536 possible state values cause problems for a table-based agent?

---

## Common Issues

**"I'm not sure what counts as part of the agent vs the environment."**
A useful rule of thumb: if the agent's code *controls* it, it's part of the agent. If it just *observes* it, the observable thing is in the environment. The line can be blurry, but for our robot, everything that runs on the Pico is the agent; everything around and under the robot is the environment.

**"What if the state doesn't fully describe the situation?"**
In the real world, it often doesn't. A one-dimensional ultrasonic reading tells you about what's directly ahead, but nothing about what's to the side. This is called a **partially observable** environment. Tabular Q-learning handles partial observability by treating whatever state you give it as complete. That's an approximation — but it works well enough for our purposes, and we'll revisit this in lesson 11.

---

Next up: designing the reward function — the signal that teaches your robot what "good" actually means.

---
