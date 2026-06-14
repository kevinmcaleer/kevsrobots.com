---
layout: lesson
title: Beyond Predefined Steps
author: Kevin McAleer
type: page
cover: assets/cover.jpg
date: 2026-06-13
previous: 00_intro.html
next: 02_anatomy_of_robotic_rl.html
description: Why hardcoded robot routines fail in dynamic environments, and how trial-and-error
  learning offers a better path.
percent: 12
duration: 5
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

Let's start with an honest conversation about the robots most of us have built.

You write a `forward()` function. You write a `turn_left()`. You carefully measure the obstacle threshold — "if the sensor reads less than 20 cm, turn right". You test it on your kitchen table. It works! You put it on the floor of the living room. It crashes into the sofa leg. You tweak the threshold to 25 cm. Now it spins in the corner for thirty seconds before eventually escaping. You stay up until midnight adding more `if` statements.

Sound familiar?

---

## The Hardcoded Approach

The typical beginner robot loop looks something like this:

```python
def navigate():
    while True:
        distance = get_distance()

        if distance < 20:
            stop()
            turn_right()
            sleep(0.4)
        else:
            forward()

        sleep(0.1)
```

This is perfectly reasonable code. The logic is clear and it's easy to reason about. But it contains a hidden assumption: **the programmer already knows the right response to every situation the robot will encounter.**

When the world is simple and predictable, that is fine. When the world is messy — different floor surfaces, objects of varying heights, variable lighting, a curious cat — the assumption breaks down fast.

Every new situation requires a new rule. Every new rule might conflict with an existing one. The codebase grows, becomes brittle, and starts hiding bugs that only appear at 2 am on a Tuesday.

---

## The Dog-Training Analogy

Think about how you'd train a dog to sit.

You don't hand the dog a rulebook. You don't reprogram its brain. You create a feedback signal: when the dog sits, something good happens (a treat). When the dog does something you don't want, the treat doesn't come, or a gentle "no" signals disapproval.

Over many repetitions, the dog figures out the strategy on its own. The behaviour emerges from feedback, not from instructions.

Reinforcement learning is exactly this idea, applied to software:

- The **robot** is the dog
- The **task** (obstacle avoidance, reaching a goal) is the trick to learn
- The **reward signal** is the treat
- **Trial and error over many episodes** replaces the training sessions

The robot's code does not contain the answer at the start. The answer is discovered through experience.

---

## Dynamic Environments Need Adaptive Policies

Here's the key insight: in a static, fully known environment, hardcoded rules can be optimal. In a *dynamic* environment — one where the layout changes, sensor readings are noisy, or the task evolves — a fixed rule set will always be playing catch-up.

A learned policy doesn't just execute a script. It evaluates the current situation and chooses the action that has worked best in the past for situations that looked like this one. When you train it in a varied enough environment, it generalises gracefully to situations it hasn't seen before.

This is a genuine paradigm shift. Let's be concrete about the difference:

| Hardcoded approach | Learned approach |
|---|---|
| You write the rules | The robot discovers the rules |
| Fails in new situations | Generalises to new situations |
| Debugging is adding more if-statements | Debugging is adjusting rewards |
| Fast to implement for simple tasks | Takes longer to set up, scales further |
| Transparent: you can read the logic | Interpretable via the Q-table |
{:class="table table-single"}


For simple, well-defined environments, hardcoded logic is often the right choice — don't reach for RL when a `if distance < 20: turn_right()` genuinely solves the problem. But once the environment gets complicated enough that you're managing dozens of rules and still seeing failures, it's worth learning a better tool.

---

## A Taste of What's Coming

By lesson 10 of this course, you'll have a Python program that does something like this:

```python
# No hardcoded rules about what to do in each situation.
# The robot discovers the best action by trying many thousands
# of moves in a simulated grid world and receiving rewards.

for episode in range(500):
    state = env.reset()
    total_reward = 0

    while not done:
        action = choose_action(q_table, state, epsilon)
        next_state, reward, done = env.step(action)
        q_table = update_q(q_table, state, action, reward, next_state)
        state = next_state
        total_reward += reward

    print(f"Episode {episode}: total reward = {total_reward:.1f}")
```

Each episode, the robot tries to navigate a grid, gets rewards for good moves and penalties for collisions, and updates a table of "how good is each action in each situation?" values. After enough episodes, that table holds a surprisingly competent policy — no rules written by hand.

---

## Try It Yourself

Before moving on, have a think about your own robot projects:

1. Write down one situation where your hardcoded logic failed or behaved unexpectedly. What would the robot have needed to "know" to handle it correctly?
2. Imagine a reward signal for the same task. What would earn a positive reward? What should earn a negative one?
3. What are three different "situations" (states) your robot might find itself in? How would you describe them in numbers?

These questions don't have right or wrong answers yet — we're just starting to think in RL terms. Keep your answers in mind as we work through the next few lessons.

---

Next up, we'll give all these ideas precise names and pin down exactly what "agent", "environment", "state", and "action" mean in a robotics context.

---
