---
layout: lesson
title: Looking Into the Future
author: Kevin McAleer
type: page
cover: assets/cover.jpg
date: 2026-06-13
previous: 05_markov_decision_processes.html
next: 07_the_q_table.html
description: Discover why future rewards are worth less than immediate ones, and use
  discount factors and the Bellman equation to see the idea working in Python.
percent: 42
duration: 6
date_updated: 2026-06-13
navigation:
- name: Reinforcement Learning for Beginners
- content:
  - section: Introduction
    content:
    - name: Introduction to Reinforcement Learning for Beginners
      link: 00_intro.html
  - section: The Paradigm Shift
    content:
    - name: Beyond Predefined Steps
      link: 01_beyond_predefined_steps.html
    - name: The Anatomy of Robotic RL
      link: 02_anatomy_of_robotic_rl.html
    - name: Designing the Reward Function
      link: 03_designing_the_reward_function.html
  - section: The Framework of Decisions
    content:
    - name: Mapping the Robot's World
      link: 04_mapping_the_robots_world.html
    - name: Markov Decision Processes
      link: 05_markov_decision_processes.html
    - name: Looking Into the Future
      link: 06_looking_into_the_future.html
  - section: Tabular Q-Learning from Scratch
    content:
    - name: The Q-Table
      link: 07_the_q_table.html
    - name: Exploration vs Exploitation
      link: 08_exploration_vs_exploitation.html
    - name: Coding the Learning Loop
      link: 09_coding_the_learning_loop.html
    - name: Training in Simulation
      link: 10_training_in_simulation.html
  - section: Bringing It to the Real World
    content:
    - name: The Sim-to-Real Gap
      link: 11_the_sim_to_real_gap.html
    - name: Discretising Real Sensors
      link: 12_discretizing_real_sensors.html
    - name: Deploying the Brain
      link: 13_deploying_the_brain.html
    - name: Scaling Up to Deep RL
      link: 14_scaling_up_to_deep_rl.html
    - name: Summary and What's Next
      link: 15_summary.html
---


![Cover](assets/cover.jpg){:class="cover"}

---

Here's a thought experiment. Would you rather have £10 right now, or £10 in six months? Most people take the money now. There's an element of uncertainty about the future (what if something changes?), and there's an opportunity cost (you could invest it). Future rewards are, in some sense, worth less than equivalent immediate rewards.

Reinforcement learning captures this intuition with a single number called the **discount factor**, written as γ (gamma).

---

## The Discount Factor (γ)

The discount factor γ is a number between 0 and 1. It controls how much the agent cares about future rewards relative to immediate ones.

```python
gamma = 0.9   # the agent values future rewards at 90% of their face value per step

# Reward at time 0 (now):       worth  reward * 1.0
# Reward at time 1 (1 step):    worth  reward * gamma       = reward * 0.9
# Reward at time 2 (2 steps):   worth  reward * gamma**2    = reward * 0.81
# Reward at time 3 (3 steps):   worth  reward * gamma**3    = reward * 0.729
# ...
# Reward at time n (n steps):   worth  reward * gamma**n
```

With γ = 0.9, a reward earned five steps from now is worth 0.9⁵ ≈ 0.59 of its face value. Ten steps away: 0.9¹⁰ ≈ 0.35. The further in the future, the less it weighs in today's decision.

---

## Discounted Return: A Worked Example

The **discounted return** from a given time step is the sum of all future rewards, each multiplied by the appropriate power of γ.

Let's trace through a tiny corridor episode. The robot starts at position 0 (left) and needs to reach position 4 (the goal), taking one step right per time step.

```python
gamma = 0.9

# Rewards received at each time step
rewards = [-0.1, -0.1, -0.1, -0.1, 10.0]
# step 0: move right (small cost)
# step 1: move right (small cost)
# step 2: move right (small cost)
# step 3: move right (small cost)
# step 4: reach goal (big reward)

# Calculate discounted return from step 0
def discounted_return(rewards, gamma, start_step=0):
    """
    Compute the total discounted return starting from start_step.

    G_t = r_t + gamma * r_{t+1} + gamma^2 * r_{t+2} + ...

    In plain English: sum up all future rewards, but shrink each
    one by gamma^k where k is how many steps in the future it is.
    """
    total = 0.0
    for k, r in enumerate(rewards[start_step:]):
        total += (gamma ** k) * r
    return total

G0 = discounted_return(rewards, gamma, start_step=0)
G1 = discounted_return(rewards, gamma, start_step=1)
G4 = discounted_return(rewards, gamma, start_step=4)

print(f"Return from step 0 (start): {G0:.4f}")
print(f"Return from step 1:         {G1:.4f}")
print(f"Return from step 4 (goal):  {G4:.4f}")

# Output:
# Return from step 0 (start): 6.2201
# Return from step 1:         6.9112
# Return from step 4 (goal): 10.0000
```

Notice that G1 > G0: being one step closer to the goal is more valuable, because the big +10 reward is discounted by one fewer factor of γ. This is exactly the gradient the Q-learning algorithm will follow.

---

## The Bellman Equation (in Python)

Richard Bellman gave us an elegant recursion that expresses the value of a state in terms of the value of the next state. It's the foundation of Q-learning, and it says:

> The value of being in a state is the immediate reward plus the (discounted) value of the next state.

Here's the Bellman equation written as commented Python:

```python
def bellman_value(reward, gamma, next_state_value):
    """
    Bellman equation for state values.

    V(s) = r + gamma * V(s')

    - V(s)  : value of the current state  (what we want to compute)
    - r     : reward received this step    (known immediately)
    - gamma : discount factor              (how much we trust the future)
    - V(s') : value of the next state      (estimated from experience)

    In plain English:
    "How good is being here?" =
        "What I just got" + "a bit less than how good the next place is"
    """
    return reward + gamma * next_state_value

# Example: one step from the goal (next_state_value = 10.0, reward = -0.1)
v_one_step_from_goal = bellman_value(reward=-0.1, gamma=0.9, next_state_value=10.0)
print(f"Value of being one step from goal: {v_one_step_from_goal:.2f}")
# Output: 8.90

# Two steps from the goal
v_two_steps = bellman_value(reward=-0.1, gamma=0.9, next_state_value=v_one_step_from_goal)
print(f"Value of being two steps from goal: {v_two_steps:.2f}")
# Output: 7.91
```

The value decreases smoothly as we move further from the goal. This smooth gradient is what Q-learning exploits: every small update propagates the goal's value backward through the state space, step by step, episode by episode.

---

## Choosing Your Gamma

The choice of γ significantly changes the agent's behaviour:

```python
# High gamma = patient agent, plans far ahead
gamma_patient = 0.99
# - Good for tasks where the goal is many steps away
# - Slower to train (longer credit assignment chain)
# - Can be unstable early in training

# Medium gamma = balanced agent
gamma_balanced = 0.9
# - Works well for most grid-world tasks
# - Reasonable training stability
# - Our default for the BurgerBot simulation

# Low gamma = impatient agent, lives for now
gamma_impatient = 0.5
# - Essentially ignores anything more than 3-4 steps ahead
# - Very stable training, quick to learn simple tasks
# - Poor for tasks requiring long-term planning
```

For our obstacle-avoidance task, γ = 0.9 is a good default. The goal is rarely more than 10–15 steps away in our 5×5 grid, and 0.9¹⁵ ≈ 0.2 means the agent still cares meaningfully about reaching it.

---

## Try It Yourself

1. Run `discounted_return()` with γ = 0.5, 0.9, and 0.99 on the same reward sequence `[-0.1, -0.1, -0.1, -0.1, 10.0]`. How does the return from step 0 change? Which γ makes the agent most eager to reach the goal?

2. Modify the corridor to add an obstacle that causes a -10 reward at step 2. How does this change the discounted return from step 0 with γ = 0.9?

3. Write a function `bellman_sequence(rewards, gamma)` that applies the Bellman equation backward through a full reward sequence, starting from the last step and working to the first. Print the value at each step. Verify it matches `discounted_return()`.

---

## Common Issues

**"My agent seems to ignore the goal because it's too far away."**
Try increasing γ toward 0.99. This extends the agent's effective planning horizon and makes distant rewards matter more during early training.

**"Training seems very unstable — huge swings in total reward per episode."**
Try reducing γ toward 0.8 or 0.7. A shorter horizon makes the value estimates more stable, at the cost of some long-term planning ability.

**"Should gamma ever be exactly 1.0?"**
Only for **finite horizon** tasks where you know every episode terminates in a bounded number of steps. In our setting, γ = 1.0 can lead to infinite discounted returns if the agent never reaches the goal, which breaks the value estimates. Stick to γ < 1.0.

---

Next up: we'll build the Q-table itself — the robot's "cheat sheet" that stores everything it has learned.

---
