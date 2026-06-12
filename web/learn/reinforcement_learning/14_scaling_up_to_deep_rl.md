---
layout: lesson
title: Scaling Up to Deep RL
author: Kevin McAleer
type: page
cover: assets/cover.jpg
date: 2026-06-13
previous: 13_deploying_the_brain.html
next: 15_summary.html
description: Discover where lookup tables run out of road, how neural networks take
  over as Q-function approximators, and where to go next.
percent: 90
duration: 7
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

You've built something real: a robot that learned its behaviour from experience. Tabular Q-learning is a complete, working, understandable solution for problems where the state space is small enough to fit in a dictionary.

But what happens when it isn't?

This lesson takes an honest look at where tabular methods break down and introduces the intuition behind **Deep Q-Learning (DQN)** — the extension that scales to much harder problems. We won't implement it here (it needs a proper deep learning framework), but you'll understand what it is and what you'd need to learn next.

---

## The Scaling Wall

Recall the state-space size calculation from lesson 4. Our BurgerBot simulation used:

```python
# 7x7 grid × 4 headings × 3 distance bands = 588 possible states
total_states = 7 * 7 * 4 * 3   # = 588

# With 4 actions: 588 * 4 = 2352 Q-table entries
# Easily fits in RAM, trained in minutes
```

Now imagine a robot with a camera instead of an ultrasonic sensor. A 64×64 greyscale image has 4,096 pixels, each with 256 possible values. The number of possible states is 256^4096 — a number so large that even if you used every atom in the observable universe as a memory cell, you couldn't store one row of that table.

This is not a memory-engineering problem — it is a fundamental limit of the lookup-table approach.

```python
# State space for a camera-equipped robot:
camera_states = 256 ** (64 * 64)  # incomprehensibly large
# A lookup table is impossible.
# We need a function that *generalises* across similar states.
```

---

## Neural Networks as Q-Function Approximators

The insight behind Deep RL is this: instead of storing Q-values in a lookup table, **approximate the Q-function with a neural network**.

```python
# Tabular Q-learning:
# Q(state, action) = lookup q_table[state][action]

# Deep Q-learning (DQN):
# Q(state, action) = neural_network.predict(state)[action_index]
```

The neural network takes a state (e.g. a raw pixel image) as input and outputs a Q-value for each possible action. You train it with gradient descent, using the same Bellman target we used in lesson 6 and 9:

```python
# DQN loss (conceptual Python — not runnable without a deep learning library):
def dqn_loss(network, state, action, reward, next_state, gamma):
    """
    Compute the loss for one experience.

    The network predicts Q-values for all actions given a state.
    We compare its prediction for the taken action against the
    Bellman target, and compute mean-squared error.
    """
    # Predict Q-values for all actions in current state
    predicted_q_values = network.predict(state)
    predicted_q = predicted_q_values[action]

    # Compute Bellman target (same formula as lesson 9)
    next_q_values = network.predict(next_state)
    td_target = reward + gamma * max(next_q_values)

    # Loss: squared TD error
    loss = (td_target - predicted_q) ** 2
    return loss

# Gradient descent nudges the network's weights to reduce this loss
# over many thousands of training steps.
```

The magic: once trained, the network **generalises**. Two pixel images that look similar will produce similar Q-value estimates, even if the agent has never seen exactly that image before. A lookup table can't do this — it treats every new state as completely foreign.

---

## Where DQN Came From

In 2013, DeepMind published a paper showing that a single DQN agent — given only raw pixels and a game score — could learn to play 49 Atari games at human or superhuman level. The state space (210×160 pixels, 128 colours) is astronomically larger than anything a lookup table could handle.

The key ideas that made it work:

**Experience replay**: instead of learning from each experience immediately, store experiences in a buffer and sample them randomly during training. This breaks correlations between consecutive observations and stabilises training.

**Target network**: use a second, slower-updating copy of the network to compute the Bellman target. Without this, the target moves every step, making training unstable.

**Convolutional layers**: the network processes the raw pixels through convolutional layers (the same architecture used in image recognition) that learn to detect relevant visual features automatically.

None of these are needed for tabular Q-learning — but all three are essential for DQN to work reliably.

---

## The RL Landscape Beyond DQN

DQN was a landmark, but the field has kept moving:

| Algorithm | What it adds | When to use it |
|---|---|---|
| Double DQN | Reduces overestimation of Q-values | Most DQN tasks |
| Dueling DQN | Separate value and advantage streams | Sparse rewards |
| PPO | Policy gradient instead of Q-learning | Continuous actions |
| SAC | Off-policy + entropy bonus for exploration | Robotics with continuous control |
| AlphaGo/MuZero | Model-based: learns to plan using a learned world model | Complex strategy |

For robotics tasks with **continuous action spaces** (e.g. direct torque control of servo motors rather than "turn left / turn right"), policy gradient methods like **PPO** (Proximal Policy Optimisation) and **SAC** (Soft Actor-Critic) generally work better than DQN-style methods.

---

## What You'd Need to Learn Next

To implement DQN yourself, the skills to acquire next are:

1. **NumPy** — efficient numerical arrays; almost all deep learning code uses it
2. **A deep learning framework** — PyTorch is the most popular for research; TensorFlow/Keras is common in production
3. **OpenAI Gymnasium** — the standard environment API for RL training (similar to our `GridWorld` class but standardised)
4. **A GPU** (or Google Colab) — DQN training on Atari can take millions of steps; a GPU makes this practical

A good first DQN project: train a DQN to play CartPole (a simple balancing task) using PyTorch and the Gymnasium `CartPole-v1` environment. It's the "hello world" of deep RL and achieves a working solution in hundreds of thousands of steps on a laptop CPU.

---

## Tabular vs Deep: Knowing Which to Reach For

Before upgrading to deep RL, ask:

```python
def should_i_use_deep_rl(task):
    """
    A rough decision guide.
    """
    state_is_images_or_raw_sensors = True   # e.g. camera, lidar point cloud
    state_space_too_large_for_table = True  # > ~10,000 states
    need_continuous_actions = False         # e.g. exact servo angle vs 4 directions
    have_compute_and_time = False           # GPU + days of training

    if (not state_is_images_or_raw_sensors and
        not state_space_too_large_for_table and
        not need_continuous_actions):
        return "Tabular Q-learning — simpler, faster, more interpretable"

    if have_compute_and_time:
        return "Deep RL (DQN, PPO, SAC) — scales to complex problems"

    return "Reconsider the state space — can you discretise more aggressively?"
```

For most maker-scale robot projects, tabular Q-learning is the right choice. The Q-table is readable — you can print it, inspect it, and understand *why* the robot does what it does. That interpretability is genuinely valuable.

---

## Try It Yourself

1. Increase the grid size to 10×10 with more obstacles and retrain. At what point does training time become impractical? Estimate the maximum grid size you could train comfortably on your laptop in under 5 minutes.

2. Look up the Gymnasium `FrozenLake-v1` environment (a 4×4 or 8×8 grid-world task). It has the same structure as our `GridWorld`. Try adapting your Q-learning code to use it — the only change is calling `env.step()` and `env.reset()` in the standard Gymnasium API style.

3. Read the abstract and first page of the original DQN paper: "Human-level control through deep reinforcement learning", Mnih et al., Nature 2015. What did the authors claim was new about their approach? How does it compare to what you've built in this course?

---

## Common Issues

**"I tried PyTorch DQN tutorials and they don't look like the code in this course."**
That's expected. DQN requires a lot of scaffolding (experience replay buffer, target network, batch training) that tabular Q-learning doesn't need. The *core idea* (Bellman target, TD error, greedy action selection) is identical — but the implementation is much more involved.

**"Is there a way to run DQN on the Pico?"**
Not practically. DQN requires floating-point matrix operations (the neural network forward pass) that are too slow on the Pico's microcontroller. The Pico is excellent for *executing* a trained tabular policy; Deep RL training must happen on a laptop or cloud GPU, with only the final model deployed to embedded hardware.

---

Next up: the summary — a recap of everything you've built, a key concepts checklist, and pointers to what to explore next.

---
