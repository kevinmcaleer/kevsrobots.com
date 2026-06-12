---
layout: lesson
title: The Sim-to-Real Gap
author: Kevin McAleer
type: page
cover: assets/cover.jpg
date: 2026-06-13
previous: 10_training_in_simulation.html
next: 12_discretizing_real_sensors.html
description: "Understand why trained policies sometimes fail on real hardware \u2014\
  \ and practical strategies to bridge the gap before you deploy."
percent: 72
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

You've just trained a beautiful Q-table. The simulated BurgerBot navigates the grid flawlessly. You copy the JSON file to your Pico, plug in the batteries, and... the robot drives straight into the first chair leg it encounters.

Welcome to the **sim-to-real gap** — one of the most honest lessons in applied robotics. The simulation was a model of reality, and every model is wrong in at least some ways. When those ways matter, the policy breaks.

This lesson is about understanding *why* the gap exists and *what you can do about it* before you ever power up the hardware.

---

## Why Policies Fail on Real Hardware

### 1. Wheel Slip and Motor Mismatch

In simulation, "turn left" rotates the robot exactly 90°. On a real floor, it might rotate 85° or 95° depending on:

- The floor surface (carpet vs tiles vs laminate)
- Battery voltage (motors spin faster when fresh, slower when depleted)
- Whether the two motors are perfectly matched (they rarely are)
- Whether the wheels have good grip

After a few turns, the robot's actual heading has drifted from what it thinks its heading is. States that assume "I'm facing east" now correspond to "I'm actually facing south-east", and the Q-table has no entry for that.

### 2. Sensor Noise

The HC-SR04 is a reasonable sensor, but it has quirks:

- Reflections at oblique angles can miss the sensor entirely (the sound scatters away)
- Very close objects (<4 cm) produce unreliable readings
- Some surfaces absorb ultrasound (soft furnishings, fabric)
- Occasionally the echo never returns and the sensor reports maximum range

In simulation, the sensor is perfect. In reality, you'll get occasional wild readings that map to the wrong distance band.

### 3. Timing and Latency

In simulation, each "step" is instant. On real hardware:

- `get_distance()` takes ~20 ms
- Motor commands take ~5 ms to reach the driver
- The robot is still moving while the Pico is computing
- A fast-moving robot may cover 5 cm between sensor reading and the motor responding

At low speeds this matters less. At full speed it matters a lot.

### 4. State Aliasing

Our discrete state "near / medium / far" compresses a lot of information into three buckets. Two physically different situations might produce the same state label, causing the agent to apply the same policy to situations that actually require different responses.

---

## Mitigations You Can Apply Right Now

### Train With Sensor Noise

The most effective sim-to-real mitigation is to make the simulation less perfect — add noise to the sensor model so the policy is trained to handle uncertainty:

```python
import random

def _read_sensor_with_noise(self):
    """
    Simulated sensor with added noise.

    Occasionally returns a random reading to simulate
    bad reflections, dropouts, and other real-world artefacts.
    """
    # 10% chance of a completely bogus reading
    if random.random() < 0.10:
        return random.choice(["near", "medium", "far"])

    # Normal reading
    return self._read_sensor_clean()
```

This small change forces the Q-learning agent to develop a policy that is robust to occasional misreadings — it can't rely on the sensor always being right.

### Add Heading Uncertainty

Simulate motor mismatch by randomly perturbing the heading after a turn:

```python
def _execute_turn(self, direction):
    """
    Turn the robot, but occasionally overshoot or undershoot.
    Models real-world motor mismatch.
    """
    idx = HEADINGS.index(self.heading)
    if direction == "left":
        delta = -1
    else:
        delta = +1

    # 5% chance of turning an extra 90° (missed turn)
    if random.random() < 0.05:
        delta *= 2

    self.heading = HEADINGS[(idx + delta) % 4]
```

> **Note:** adding too much noise makes the task unsolvable — start at 5–10% perturbation probability and observe whether the agent can still learn.

### Use Conservative Distance Thresholds

The discretisation thresholds you set in simulation determine when "near" triggers. On the real robot, tighten the "near" threshold:

```python
# In simulation (faster reaction time, no lag):
NEAR_THRESHOLD   = 1   # 1 grid cell = very close

# On real hardware (more cautious — stop turning sooner):
NEAR_THRESHOLD_REAL = 12   # cm — bigger safety margin
```

A policy trained to react to "near" at 1 simulated cell will still work if the real hardware triggers "near" at 12 cm, because the Q-table entry is the same. You're just being more conservative about *when* you say "this situation counts as near."

### Add a Safety Fallback

Even with a well-trained policy, add a hardcoded safety override in the deployment code:

```python
def choose_action_safe(q_table, state, distance_cm):
    """
    Q-table lookup with a safety override.

    If the raw distance is critically small (< 8 cm),
    override the policy with an emergency stop regardless
    of what the Q-table recommends.
    """
    if distance_cm < 8:
        return "stop"   # unconditional emergency stop

    return choose_action_greedy(q_table, state)
```

This catches the cases where sensor noise or timing lag allows the robot to get dangerously close before the Q-table would ordinarily intervene.

---

## The Transfer Checklist

Before deploying to the real robot, work through this checklist:

```
[ ] Retrain with sensor noise (10% random readings)
[ ] Retrain with occasional heading perturbation (5%)
[ ] Conservative near/medium thresholds set for real hardware
[ ] Safety fallback implemented for < 8 cm readings
[ ] Battery is fully charged (motor behaviour changes as it depletes)
[ ] Robot tested on the actual floor surface it will navigate
[ ] Open space for first test run (not a cluttered room)
[ ] Power-off procedure confirmed (don't let it run into walls indefinitely)
```

---

## Try It Yourself

1. Add the sensor noise function to your `GridWorld` class (replace `_read_sensor` with the noisy version). Retrain and compare the Q-table to the noise-free version. Are the best actions different? Is the policy more or less consistent?

2. Add heading perturbation and retrain. Does the agent need more or fewer episodes to converge? Does the final average reward change significantly?

3. Write a function `simulate_noisy_transfer(q_table, noise_level, n_episodes=100)` that tests the trained Q-table in a noisy environment (without further training) and reports the success rate (percentage of episodes that reach the goal). Try noise levels of 0%, 10%, 20%, 30%. At what noise level does the policy break down?

---

## Common Issues

**"My robot curves instead of going straight."**
Motor mismatch. Add a small PWM offset to the slower motor's duty cycle. This is a calibration step, not an RL fix.

**"The robot turns fine but ends up in completely the wrong direction."**
Systematic heading error — likely one motor is significantly faster. Try adding a gyroscope or encoder feedback. Alternatively, use slightly longer turn durations in the MicroPython code and accept the approximation.

**"The robot works on carpet but not on smooth tiles."**
Slippage is much worse on smooth surfaces. Either reduce speed (more controlled turns) or retrain with higher heading perturbation probability.

**"The HC-SR04 keeps reading maximum range even when there's a wall 10 cm away."**
Check the ECHO pin voltage. The 5V version of the HC-SR04 will output 5V on ECHO, which can damage the Pico. Use a voltage divider (1kΩ and 2kΩ resistors) or a 3.3V compatible sensor.

---

Next up: practical MicroPython code for reading the HC-SR04 reliably and mapping its output to the same state bins we used in training.

---
