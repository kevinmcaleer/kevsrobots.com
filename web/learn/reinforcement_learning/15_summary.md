---
layout: lesson
title: Summary and What's Next
author: Kevin McAleer
type: page
cover: assets/cover.jpg
date: 2026-06-13
previous: 14_scaling_up_to_deep_rl.html
description: Recap the key concepts, review what you've built, and find out where
  to take your robotics and AI journey next.
percent: 100
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

You made it! Let's take a moment to look back at the journey from "my robot crashes into the sofa" to "my robot has a trained policy it discovered on its own."

---

## What You've Built

Over the course of 14 lessons, you:

1. **Understood why hardcoded rules fail** — and why trial-and-error learning is a fundamentally better approach for dynamic environments
2. **Defined the RL vocabulary** — agent, environment, state, action, reward — in terms of a physical robot
3. **Designed a reward function** — including learning to spot reward-hacking traps like the statue problem
4. **Discretised continuous sensors** — turning messy real-world readings into clean state labels a lookup table can index
5. **Understood Markov Decision Processes** — just enough theory to know why the current state is enough
6. **Implemented discount factors** — and saw the Bellman equation as a simple recursive Python function
7. **Built a Q-table from scratch** — using a Python `defaultdict` with JSON serialisation for Pico deployment
8. **Implemented ε-greedy exploration** — with epsilon decay that transitions from full exploration to mostly exploitation
9. **Coded the complete Q-learning update rule** — and understood every term: α, γ, TD target, TD error
10. **Trained in simulation** — watching the average episode reward rise from negative (crashes) to positive (goal reached) over hundreds of episodes
11. **Understood the sim-to-real gap** — and applied practical mitigations: sensor noise in training, conservative thresholds, safety fallbacks
12. **Read and filtered HC-SR04 data** — using median-of-three filtering and mapping to the same state bins as training
13. **Deployed to the Pico** — loading the JSON Q-table in MicroPython and running the greedy policy in a live control loop
14. **Looked ahead to Deep RL** — understanding where tables run out of road and what neural network approximators make possible

---

## Key Concepts Checklist

Use this as a revision guide. You should be able to explain each item in your own words:

**Core vocabulary:**
- [ ] What is a reinforcement learning agent?
- [ ] What distinguishes the environment from the agent?
- [ ] What is a state, and why do we discretise continuous readings?
- [ ] What is an action space, and what are the four actions our robot uses?
- [ ] What is a reward, and why does its design matter so much?

**The framework:**
- [ ] What does the Markov property tell us about what information we need?
- [ ] What is an episode, and what ends one?
- [ ] What does the discount factor γ control?
- [ ] What is the Bellman equation, in plain English?

**The algorithm:**
- [ ] What does a Q-value represent?
- [ ] How is a Q-table indexed?
- [ ] What is the ε-greedy strategy and why do we need it?
- [ ] What is epsilon decay and what problem does it solve?
- [ ] Write the Q-learning update rule from memory (as a Python expression with comments)

**Deployment:**
- [ ] What are three sources of the sim-to-real gap?
- [ ] How does median-of-three filtering help with real sensor noise?
- [ ] Why do we use a simplified state (heading + distance only) on the real robot?
- [ ] What is a safety fallback and why is it important?

**Beyond tabular:**
- [ ] At what point does a lookup table become impractical?
- [ ] What does a DQN's neural network approximate?
- [ ] Name two deep RL algorithms suitable for continuous action spaces

---

## The Q-Learning Update Rule (From Memory)

This is the one equation worth memorising. Here it is one more time as commented Python:

```python
# Q-learning update — commit this to memory
# Bellman-derived, Watkins 1989, still the heart of modern RL

q_table[state][action] += alpha * (
    reward                          # what I just got
    + gamma * max(q_table[next_state].values())  # discounted best future
    - q_table[state][action]        # minus my current estimate (TD error)
)

# alpha (learning rate):  how boldly I update — typically 0.1–0.3
# gamma (discount):       how much I trust the future — typically 0.9–0.99
# TD error:               positive = I was too pessimistic, negative = too optimistic
```

---

## What's Next

You've covered tabular Q-learning end-to-end. Here are the natural next steps, in roughly increasing order of difficulty:

### Stay on the Robot

**[MicroPython Robotics Projects](/learn/micropython_robotics/)**
If you haven't completed this course, it covers the hardware foundations (motors, sensors, PWM) that the deployment lessons assumed. A great place to consolidate the physical side.

**[Learn ROS](/learn/learn_ros/)**
The Robot Operating System is the industry-standard framework for building complex multi-component robots. If you want to scale from a two-wheeled Pico robot to a full robot arm or mobile platform, ROS is the next major skill to acquire.

### Go Deeper on AI

**NumPy** — before any deep learning framework, get comfortable with numerical arrays. The [Pandas and NumPy](/learn/pandas_and_numpy/) course on this site is a good starting point.

**OpenAI Gymnasium** — the standard RL environment API. Try the `FrozenLake`, `CartPole`, and `MountainCar` environments with your Q-learning code first, then graduate to DQN for `CartPole`.

**PyTorch or JAX** — the deep learning frameworks used in most modern RL research. PyTorch has excellent beginner tutorials and an active community.

**Spinning Up in Deep RL** — OpenAI's free educational resource: [spinningup.openai.com](https://spinningup.openai.com). Covers policy gradients, PPO, SAC, and more with clean implementations.

### Read the Papers

- **Q-Learning**: Watkins & Dayan, 1992 — the original paper; very readable
- **DQN**: Mnih et al., 2013/2015 — the Atari breakthrough; Nature version is open access
- **PPO**: Schulman et al., 2017 — the most widely deployed policy gradient algorithm
- **AlphaGo**: Silver et al., 2016 — combines RL with Monte Carlo tree search

---

## A Final Word

When you started this course, a robot that *learns* might have seemed like something reserved for research labs with expensive GPUs and machine-learning PhDs. I hope this course has shown you that the core ideas are genuinely accessible — a Python dictionary, a reward signal, and enough patience to let a virtual robot crash into walls several hundred times is all you really need to get started.

The same Bellman equation that powers cutting-edge robotics research is running in your `q_table[state][action] += alpha * td_error` line. The scale is different. The principle is identical.

Keep building. Keep experimenting. And keep letting your robots learn.

---

*Thank you for working through this course. If you'd like to share what you built, post it on social media and tag [@kevinmcaleer28](https://www.youtube.com/@kevinmcaleer28) — I'd love to see your trained robots in action.*

---
