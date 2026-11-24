---
layout: lesson
title: How the Duck Learned to Walk
author: Kevin McAleer
type: page
cover: /learn/open_duck_mini/assets/banners/12_how_the_duck_learned_to_walk.jpg
date: 2026-11-24
previous: 11_first_steps.html
next: 13_training_your_own_policy.html
description: Open up the Open Duck Mini's walking brain to see the policy, observations,
  actions and rewards, the imitation reward and reference motion, and how the team
  closed the sim-to-real gap
percent: 78
duration: 8
quiz_total: 12
date_updated: 2026-11-24
navigation:
- name: Build an Open Duck Mini Walking Robot
- content:
  - section: Meet the duck
    content:
    - name: Introduction
      link: 00_intro.html
    - name: What is the Open Duck Mini?
      link: 01_what_is_open_duck_mini.html
  - section: Parts and printing
    content:
    - name: The Parts List
      link: 02_parts_list.html
    - name: Printing the Parts
      link: 03_printing_the_parts.html
    - name: Serial Bus Servos and IDs
      link: 04_bus_servos_and_ids.html
  - section: Assembly
    content:
    - name: Assembling the Legs
      link: 05_assembling_the_legs.html
    - name: Head, Body and Battery
      link: 06_head_body_and_battery.html
    - name: Wiring the Electronics
      link: 07_wiring_the_electronics.html
  - section: Bring it to life
    content:
    - name: Setting up the Pi
      link: 08_setting_up_the_pi.html
    - name: Testing the IMU, Motors and Controller
      link: 09_testing_imu_motors_and_controller.html
    - name: Calibration and Offsets
      link: 10_calibration_and_offsets.html
    - name: First Steps
      link: 11_first_steps.html
  - section: The walking brain
    content:
    - name: How the Duck Learned to Walk
      link: 12_how_the_duck_learned_to_walk.html
    - name: Training Your Own Policy
      link: 13_training_your_own_policy.html
    - name: Troubleshooting
      link: 14_troubleshooting.html
    - name: Summary
      link: 15_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

Your duck walks. But nobody programmed that walk step by step. So where did it come from? In this lesson we'll open up the duck's walking brain and see how it was made. If you've done [Reinforcement Learning for Beginners](/learn/reinforcement_learning/), you'll recognise a lot of it.

---

## The four RL words, duck edition

Back in the RL course we met the agent, its observations, its actions and its reward (see [the anatomy of robotic RL](/learn/reinforcement_learning/02_anatomy_of_robotic_rl.html)). Here's how they map onto the duck.

| RL idea | On the Open Duck Mini |
|---|---|
| Policy | A neural network, saved as the `.onnx` file you ran in lesson 11 |
| Observation | Motor positions and speeds, the IMU, the foot switches, and the joystick command |
| Action | Target positions for the joints |
| Reward | A score for following the joystick command and for walking like the reference motion |
{:class="table table-single"}

In the RL course our policy was a Q-table, a lookup table of states and actions. That works for a handful of states. The duck's observation is lots of numbers that change smoothly, so a table can't hold it. Instead, the policy is a neural network, as we saw in [scaling up to deep RL](/learn/reinforcement_learning/14_scaling_up_to_deep_rl.html). It was trained with PPO, through MuJoCo Playground's Brax trainer. If you want the details, check `runner.py` in the [Open_Duck_Playground](https://github.com/apirrone/Open_Duck_Playground) repo.

---

## The imitation reward

Here's a problem. If you only reward a robot for moving forward, it'll find *a* way to move forward. It might shuffle, hop or throw itself along. RL is very good at finding loopholes in a reward (we saw this in [designing the reward function](/learn/reinforcement_learning/03_designing_the_reward_function.html)).

The Open Duck team borrowed an idea from Disney's BDX paper: an **imitation reward**. As well as the task rewards, such as following the velocity command, the policy is paid for staying close to a **reference motion**, an example of what a good walk looks like.

Where does the reference motion come from? The [Open_Duck_reference_motion_generator](https://github.com/apirrone/Open_Duck_reference_motion_generator), built on a robotics library called Placo, generates walking gaits for a range of speeds: forward and back (dx), sideways (dy) and turning (dtheta). The ranges are set in `auto_gait.json`. It then fits polynomials to those gaits and saves them as `polynomial_coefficients.pkl`, which the trainer uses as its reference motion.

So the policy learns two things at once: *go where the joystick says*, and *do it in a way that looks like this walk*.

---

## A toy imitation reward

Let's make the idea concrete with a tiny simulation. It uses only Python's standard library, so it runs on any computer. One knee joint should follow a sine wave, our "reference motion". We score four pretend policies by how closely they follow it at each step.

This is a toy. The real reward in the Playground covers every joint and adds the task rewards on top. But the core idea, *pay more when you're closer to the reference*, is the same.

```python
import math

STEPS_PER_SECOND = 50   # how often our toy "robot" picks a new knee angle
STRIDE_SECONDS = 1.0    # one full step cycle takes one second
SHARPNESS = 20.0        # how quickly the reward falls off as the error grows


def reference_knee(t):
    """The gait we want to copy: the knee swings like a sine wave (radians)."""
    return 0.5 * math.sin(2 * math.pi * t / STRIDE_SECONDS)


def imitation_reward(actual, target):
    """1.0 for a perfect match, sliding towards 0.0 as the error grows."""
    error = actual - target
    return math.exp(-SHARPNESS * error * error)


# Three pretend policies, each a rule for choosing the knee angle at time t
def copycat(t):
    return reference_knee(t)                      # follows the reference exactly


def sloppy(t):
    return 0.8 * reference_knee(t) + 0.05         # right shape, a bit small and offset


def statue(t):
    return 0.0                                    # never moves the knee at all


def late(t):
    return reference_knee(t - 0.25)               # right shape, a quarter of a step late


policies = {"copycat": copycat, "sloppy": sloppy, "statue": statue, "late": late}
steps = int(STEPS_PER_SECOND * STRIDE_SECONDS)

for name, policy in policies.items():
    total = 0.0
    for step in range(steps):
        t = step / STEPS_PER_SECOND
        total += imitation_reward(policy(t), reference_knee(t))
    average = total / steps
    bar = "#" * round(average * 40)
    print(f"{name:8} average reward {average:.2f}  {bar}")
```

Run it and you'll see something like this:

```text
copycat  average reward 1.00  ########################################
sloppy   average reward 0.87  ###################################
statue   average reward 0.27  ###########
late     average reward 0.18  #######
```

How it works:

- `reference_knee(t)` is the walk we want to copy.
- `imitation_reward()` turns the error between the real angle and the reference into a score. We square the error so being too high and too low count the same, then use `math.exp(-SHARPNESS * error * error)` so a perfect match scores 1.0 and the score drops quickly as the error grows.
- Each policy is scored at every step through one stride, and we average the scores.

Look at `late`. It moves exactly the right way, just a quarter of a step late, and it scores *worse* than `statue`, which doesn't move at all. Timing matters: an imitation reward checks *when* you move, not only *how*.

---

## Mind the sim-to-real gap

A policy trained in simulation can fail badly on a real robot, because the simulator is never perfect. We met this as the [sim-to-real gap](/learn/reinforcement_learning/11_the_sim_to_real_gap.html). The Open Duck team closed it in several ways.

| Problem | What the team did |
|---|---|
| The simulated robot must match the real shape | The Onshape CAD is turned into a simulation model with onshape-to-robot (from Rhoban) |
| Printed parts have infill, so they're lighter than solid parts | The part masses were overridden with estimates from the slicer |
| Real servos are not perfect motors | Rhoban's **BAM** (Better Actuator Models) was used to measure the STS3215. Its damping, kp, frictionloss, armature and forcerange go into the MuJoCo model |
| Gears have a little play in them | The `flat_terrain_backlash` task models backlash, the slop in the gears |
| Every real duck and floor is a bit different | **Domain randomisation** varies friction, masses and more in training, so the policy can't rely on any one exact value |
{:class="table table-single"}

The motor model is already done, so you don't need to redo it. Domain randomisation lives in `playground/common/randomize.py` in the Playground repo, if you want to see what gets varied.

Domain randomisation is a lovely idea. If the policy only ever saw one exact friction, it might learn a walk that only works at that friction. Train it across many, and it has to learn a walk that works on all of them, which includes, with luck, your floor.

---

## Try it Yourself

1. Change `SHARPNESS` to `5.0` and then to `80.0`. How do the scores change? What does a sharper reward mean for a learning robot?
2. Add a fifth policy that wobbles: `reference_knee(t) + 0.1 * math.sin(20 * t)`. Where does it land?
3. Challenge: add a second joint, an ankle with its own reference wave, and score the average of both joints.

## Common Issues

**Problem**: Thinking the duck learns on your floor.
**Solution**: Remember the learning happened earlier, in simulation. On the Pi, the policy only acts.
**Why**: Running the policy is cheap. Training it needs thousands of simulated robots and a GPU, which is lesson 13.

**Problem**: Assuming a bigger reward always means a better walk.
**Solution**: Watch the robot as well as the numbers.
**Why**: A policy can score well by exploiting a gap in the reward, which is exactly what the imitation reward helps prevent.

## Check your understanding

{% quiz %}
? What does the imitation reward pay the policy for?
- [ ] Walking as fast as possible
- [x] Staying close to a reference motion of a good walk
- [ ] Keeping the battery charged
> The imitation reward, borrowed from Disney's BDX paper, rewards the policy for moving like the reference gait, alongside the task rewards such as following the joystick.

? In the toy, why does the late policy score worse than the statue?
- [ ] It has a bug
- [x] Being out of time with the reference gives big errors for much of the stride
- [ ] The statue gets a bonus for standing still
> Moving the right way at the wrong moment is often further from the reference than not moving at all.

? What is domain randomisation for?
- [ ] Making training faster
- [x] Varying things like friction and masses so the policy works in the real world
- [ ] Choosing a random walking direction
> A policy trained across many conditions can't depend on one exact value, so it copes better with your real duck and floor.
{% endquiz %}
