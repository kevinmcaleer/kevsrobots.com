---
layout: lesson
title: Deploying the Brain
author: Kevin McAleer
type: page
cover: assets/cover.jpg
date: 2026-06-13
previous: 12_discretizing_real_sensors.html
next: 14_scaling_up_to_deep_rl.html
description: Export the trained Q-table to JSON, load it on the Pico, and run the
  complete MicroPython control loop that reads sensors, looks up the best action,
  and drives the motors.
percent: 84
duration: 9
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

This is the payoff lesson. We've trained a policy, understood its limitations, and built the sensor infrastructure. Now we put it all together: the Q-table travels from the laptop to the Pico as a JSON file, and a MicroPython script runs the greedy policy in a continuous loop, reading sensors and driving motors in real time.

Let's go.

---

## Step 1 — Retrain for the Simplified State Space

Before deploying, we need to retrain using the **simplified state** (heading + distance only, no grid position). This is necessary because the real robot can't track (row, col) without encoders.

In your `train.py`, update `_get_state()`:

```python
def _get_state(self):
    """Simplified state: heading + sensor only (no grid position)."""
    sensor = self._read_sensor()
    return (self.heading, sensor)   # 4 headings × 3 bands = 12 states
```

The Q-table now has at most 12 rows — small enough to fit comfortably on the Pico. Retrain and save:

```bash
python3 train.py
# Creates q_table.json in the current directory
```

Verify the saved file looks reasonable:

```json
{
  "('east', 'far')":   {"forward": 3.21, "turn_left": 0.15, ...},
  "('east', 'near')":  {"forward": -4.50, "turn_left": 1.21, ...},
  "('north', 'far')":  {"forward": 2.80, "turn_left": 0.10, ...},
  ...
}
```

---

## Step 2 — Copy Files to the Pico

Using Thonny (or `mpremote`):

1. Open Thonny, connect to the Pico
2. Go to **Files → Open** → choose `q_table.json` from your laptop
3. **Save to Pico** as `q_table.json`
4. Create a new file called `main.py` on the Pico with the code below

Alternatively, with `mpremote`:

```bash
mpremote connect auto cp q_table.json :q_table.json
```

---

## The Deployment Code

Save this as `main.py` on the Pico. It is the complete, standalone deployment — no training, no exploration, just the greedy policy:

```python
"""
BurgerBot RL Deployment
========================
Loads a trained Q-table from q_table.json and runs the greedy
policy on a Raspberry Pi Pico with:
  - L298N H-bridge motor driver
  - HC-SR04 ultrasonic sensor (3.3V compatible or voltage-divided)

Hardware pin assignments match the MicroPython Robotics Projects course:
  Motor driver ENA  -> GP0  (PWM speed control, left motor)
  Motor driver IN1  -> GP1  (left motor direction A)
  Motor driver IN2  -> GP2  (left motor direction B)
  Motor driver IN3  -> GP3  (right motor direction A)
  Motor driver IN4  -> GP4  (right motor direction B)
  Motor driver ENB  -> GP5  (PWM speed control, right motor)
  HC-SR04 TRIG      -> GP8
  HC-SR04 ECHO      -> GP9
"""

import json
import time
from machine import Pin, PWM, time_pulse_us

# ---------------------------------------------------------------------------
# Pin configuration
# ---------------------------------------------------------------------------

# Left motor
ENA = PWM(Pin(0)); ENA.freq(1000)
IN1 = Pin(1, Pin.OUT)
IN2 = Pin(2, Pin.OUT)

# Right motor
ENB = PWM(Pin(5)); ENB.freq(1000)
IN3 = Pin(3, Pin.OUT)
IN4 = Pin(4, Pin.OUT)

# Ultrasonic sensor
TRIG = Pin(8, Pin.OUT)
ECHO = Pin(9, Pin.IN)

# ---------------------------------------------------------------------------
# Motor constants
# ---------------------------------------------------------------------------

# Duty cycle for "normal" forward speed (0–65535 on Pico PWM)
NORMAL_SPEED = 40000   # ~61% duty cycle
TURN_SPEED   = 35000   # slightly slower for turns

# Turn duration in milliseconds — calibrate for your robot
# A 90° turn at TURN_SPEED on a smooth floor ≈ 400–600 ms
TURN_DURATION_MS = 500

# ---------------------------------------------------------------------------
# Motor control
# ---------------------------------------------------------------------------

def set_motors(left_duty, left_fwd, right_duty, right_fwd):
    """
    Set both motors simultaneously.

    left_duty  (int)  : PWM duty cycle for left motor (0–65535)
    left_fwd   (bool) : True = forward, False = reverse
    right_duty (int)  : PWM duty cycle for right motor (0–65535)
    right_fwd  (bool) : True = forward, False = reverse
    """
    # Left motor
    ENA.duty_u16(left_duty)
    IN1.value(1 if left_fwd else 0)
    IN2.value(0 if left_fwd else 1)

    # Right motor
    ENB.duty_u16(right_duty)
    IN3.value(1 if right_fwd else 0)
    IN4.value(0 if right_fwd else 1)

def motor_stop():
    ENA.duty_u16(0)
    ENB.duty_u16(0)
    IN1.value(0); IN2.value(0)
    IN3.value(0); IN4.value(0)

def motor_forward():
    set_motors(NORMAL_SPEED, True, NORMAL_SPEED, True)

def motor_turn_left():
    """Spin in place: left motor backward, right motor forward."""
    set_motors(TURN_SPEED, False, TURN_SPEED, True)
    time.sleep_ms(TURN_DURATION_MS)
    motor_stop()
    time.sleep_ms(50)   # brief pause before next command

def motor_turn_right():
    """Spin in place: left motor forward, right motor backward."""
    set_motors(TURN_SPEED, True, TURN_SPEED, False)
    time.sleep_ms(TURN_DURATION_MS)
    motor_stop()
    time.sleep_ms(50)

# ---------------------------------------------------------------------------
# Ultrasonic sensor
# ---------------------------------------------------------------------------

def get_distance_cm():
    """Return distance in cm, or -1.0 on timeout."""
    TRIG.low()
    time.sleep_us(2)
    TRIG.high()
    time.sleep_us(10)
    TRIG.low()

    duration = time_pulse_us(ECHO, 1, 30000)
    if duration < 0:
        return -1.0
    return duration / 58.0

def get_distance_filtered():
    """Median of three readings for noise rejection."""
    readings = []
    for _ in range(3):
        d = get_distance_cm()
        if d > 0:
            readings.append(d)
        time.sleep_ms(20)

    if not readings:
        return -1.0

    readings.sort()
    return readings[len(readings) // 2]

def discretize_distance(distance_cm):
    """
    Map raw distance to the state label used during training.
    Thresholds must match train.py.
    """
    if distance_cm < 0:
        return "near"   # conservative: treat failure as obstacle
    if distance_cm < 15:
        return "near"
    elif distance_cm < 40:
        return "medium"
    else:
        return "far"

# ---------------------------------------------------------------------------
# Heading tracking
# ---------------------------------------------------------------------------

HEADINGS = ["north", "east", "south", "west"]
_heading_idx = 1   # start facing east

def get_heading():
    return HEADINGS[_heading_idx]

def update_heading(direction):
    global _heading_idx
    if direction == "left":
        _heading_idx = (_heading_idx - 1) % 4
    else:
        _heading_idx = (_heading_idx + 1) % 4

# ---------------------------------------------------------------------------
# Q-table loading
# ---------------------------------------------------------------------------

def load_q_table(filename="q_table.json"):
    """
    Load Q-table from JSON. Keys are stored as strings; convert back
    to tuples using eval() — safe here because we generated the file.
    """
    with open(filename, "r") as f:
        raw = json.load(f)

    q_table = {}
    for k, v in raw.items():
        # Key looks like "('east', 'far')" — evaluate it as a Python literal
        # MicroPython doesn't have ast.literal_eval, but the format is safe
        # (we generated the file ourselves from trusted training code)
        key = eval(k)   # noqa: S307 — trusted source only
        q_table[key] = v
    return q_table

# ---------------------------------------------------------------------------
# Policy execution
# ---------------------------------------------------------------------------

def choose_action_greedy(q_table, state):
    """
    Return the action with the highest Q-value.
    Falls back to 'forward' if state not found in table.
    """
    if state not in q_table:
        print(f"Unknown state {state} — defaulting to 'forward'")
        return "forward"

    action_values = q_table[state]
    return max(action_values, key=lambda a: action_values[a])

def execute_action(action):
    """Execute an action on the real motors and update heading."""
    if action == "forward":
        motor_forward()
    elif action == "turn_left":
        motor_stop()
        motor_turn_left()
        update_heading("left")
    elif action == "turn_right":
        motor_stop()
        motor_turn_right()
        update_heading("right")
    elif action == "stop":
        motor_stop()
        time.sleep_ms(200)

def get_state():
    """Compose the current state tuple for Q-table lookup."""
    distance_cm = get_distance_filtered()
    band = discretize_distance(distance_cm)
    heading = get_heading()
    return (heading, band)

# ---------------------------------------------------------------------------
# Safety override
# ---------------------------------------------------------------------------

EMERGENCY_STOP_CM = 8   # stop unconditionally if closer than this

def safety_check(distance_cm):
    """
    Return True if the robot is too close to stop, False otherwise.
    Call before executing any forward action.
    """
    return distance_cm < EMERGENCY_STOP_CM and distance_cm > 0

# ---------------------------------------------------------------------------
# Main control loop
# ---------------------------------------------------------------------------

def run():
    """
    Load the Q-table and run the greedy policy continuously.
    Press Ctrl+C to stop.
    """
    print("Loading Q-table...")
    try:
        q_table = load_q_table("q_table.json")
    except OSError:
        print("ERROR: q_table.json not found on Pico! Copy it first.")
        return

    print(f"Q-table loaded: {len(q_table)} states")
    print("Starting policy execution. Ctrl+C to stop.")
    time.sleep_ms(1000)   # brief pause before starting

    step = 0
    try:
        while True:
            # 1. Read sensor
            distance_cm = get_distance_filtered()

            # 2. Safety override
            if safety_check(distance_cm):
                print(f"SAFETY STOP: {distance_cm:.1f} cm")
                motor_stop()
                time.sleep_ms(500)
                # Force a turn to escape
                motor_turn_right()
                update_heading("right")
                continue

            # 3. Get state and look up best action
            state = get_state()
            action = choose_action_greedy(q_table, state)

            # 4. Execute action
            execute_action(action)

            # 5. Log to console
            print(f"Step {step:4d} | dist={distance_cm:5.1f}cm | "
                  f"state={state} | action={action}")
            step += 1

            # 6. Brief pause between control cycles
            time.sleep_ms(100)

    except KeyboardInterrupt:
        motor_stop()
        print("\nStopped.")

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

run()
```

---

## Running It

1. Copy `q_table.json` and `main.py` to the Pico using Thonny
2. In Thonny, press **Run** (or press the Pico's reset button if `main.py` is the startup file)
3. The robot will pause for 1 second, then begin executing the policy
4. Watch the Thonny console for step-by-step state and action logs
5. Press **Ctrl+C** (or the Stop button in Thonny) to halt

---

## Calibrating Turn Duration

The `TURN_DURATION_MS = 500` constant controls how long the robot spins to execute a 90° turn. This value varies by robot:

```python
# To calibrate: test turns manually
# Start with 500 ms. If the robot turns less than 90°, increase.
# If it overshoots, decrease. Aim for 85–95° — close enough for the policy.

# Quick calibration test (run in Thonny REPL):
motor_turn_right()   # should turn approximately 90° clockwise
```

---

## Try It Yourself

1. Run the robot in a large, clear space first. Observe which actions it takes most often. Does the distribution of actions make sense given the Q-table?

2. Place a single obstacle (a box) directly in front of the robot at 30 cm. Observe how it detects and avoids it. Move the obstacle to different positions and watch the policy adapt.

3. Modify `run()` to keep a count of successful forward steps and turns. After 200 steps, print the ratio. A good policy should show more than 50% forward steps (the robot is making progress, not just spinning).

---

## Common Issues

**"The robot just spins in circles."**
The Q-table may not have good coverage of the heading + "far" states. Retrain with more episodes (800+) or check that the state used in training matches the state in `get_state()` exactly.

**"It hits an obstacle before the safety fallback triggers."**
Increase `EMERGENCY_STOP_CM` from 8 to 12 cm, and shorten the `TURN_DURATION_MS` so turns complete faster.

**"eval() raises an error when loading the Q-table."**
The key string format from `save_q_table()` on your laptop must exactly match what `eval()` expects. Check that strings inside the tuple use single quotes (Python's default `repr()` uses them). Alternatively, use the `ast.literal_eval()` equivalent on MicroPython: the `ujson` module parses JSON fine but can't handle tuple-as-string keys directly — this is why we use `eval()` in the deployment code.

**"The motors don't move at all."**
Check your motor driver wiring and that the PWM duty cycle isn't 0 (the Pico PWM duty_u16 range is 0–65535, not 0–100). Verify ENA and ENB are connected and receiving PWM signal with a voltmeter or LED test.

---

Congratulations — your robot is now learning-enabled! Next up: a brief look at where tabular Q-learning runs out of road, and what Deep RL does differently.

---
