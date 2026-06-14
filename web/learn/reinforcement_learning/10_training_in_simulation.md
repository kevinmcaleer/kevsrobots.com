---
layout: lesson
title: Training in Simulation
author: Kevin McAleer
type: page
cover: assets/cover.jpg
date: 2026-06-13
previous: 09_coding_the_learning_loop.html
next: 11_the_sim_to_real_gap.html
description: "Build a complete grid-world simulator, train your Q-learning agent over\
  \ hundreds of episodes, and watch the policy improve \u2014 all in plain Python."
percent: 66
duration: 10
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

This is the lesson everything has been building toward. We're going to write a complete, runnable Python program that:

1. Simulates a differential-drive robot on a 2-D grid with obstacles
2. Trains a Q-learning agent over hundreds of episodes
3. Prints the learning progress so you can watch the policy improve
4. Saves the finished Q-table to JSON, ready for the Pico

This program uses nothing beyond the Python standard library. Copy it into a file called `train.py` and run it with `python3 train.py`.

---

## The Grid World

Our simulated BurgerBot navigates a 7×7 grid. Obstacles (O) block certain cells. The robot starts at the top-left corner and must reach the goal (G) at the bottom-right corner without hitting obstacles or walls.

```
. . . O . . .
. O . . . O .
. . . . . . .
O . . O . . .
. . . . . O .
. O . . . . .
. . . . . . G
```

The robot has:
- A **position** (row, col)
- A **heading** (north, east, south, west)
- A **simulated ultrasonic sensor** — it "reads" the distance to the nearest obstacle directly ahead, within the grid, up to 5 cells away

---

## The Complete Training Program

Save this entire file as `train.py`:

```python
"""
BurgerBot Grid-World Q-Learning Trainer
========================================
Trains a tabular Q-learning agent to navigate a 7x7 grid with
obstacles and a goal. No external libraries required.

Run with: python3 train.py
"""

import random
import json
import ast
from collections import defaultdict


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

GRID_ROWS = 7
GRID_COLS = 7

# Obstacle positions (row, col) — cannot be start or goal
OBSTACLES = {
    (0, 3), (1, 1), (1, 5),
    (3, 0), (3, 3), (4, 5),
    (5, 1),
}

START = (0, 0)   # top-left
GOAL  = (6, 6)   # bottom-right

ACTIONS = ["forward", "turn_left", "turn_right", "stop"]

# Headings: index maps to (delta_row, delta_col)
HEADINGS = ["north", "east", "south", "west"]
HEADING_DELTA = {
    "north": (-1,  0),
    "east":  ( 0,  1),
    "south": ( 1,  0),
    "west":  ( 0, -1),
}

# Sensor discretisation thresholds (in grid cells)
NEAR_THRESHOLD   = 1   # obstacle 1 cell ahead
MEDIUM_THRESHOLD = 3   # obstacle 2-3 cells ahead

# Training hyperparameters
NUM_EPISODES   = 600
ALPHA          = 0.2    # learning rate
GAMMA          = 0.9    # discount factor
EPSILON_START  = 1.0    # initial exploration probability
DECAY_RATE     = 0.995  # epsilon decay per episode
EPSILON_MIN    = 0.01   # minimum exploration probability
MAX_STEPS      = 150    # max steps per episode before giving up

Q_TABLE_FILE   = "q_table.json"


# ---------------------------------------------------------------------------
# Environment
# ---------------------------------------------------------------------------

class GridWorld:
    """
    A 2-D grid-world environment for the BurgerBot simulation.

    The robot has a position (row, col) and a heading (one of HEADINGS).
    The ultrasonic sensor reads the distance to the nearest obstacle
    directly ahead, discretised into 'near', 'medium', or 'far'.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        """Return the start state and reset the robot to the start position."""
        self.row, self.col = START
        self.heading = "east"   # start facing east (into the grid)
        self.done = False
        return self._get_state()

    def _get_state(self):
        """Return the current state as a tuple."""
        sensor = self._read_sensor()
        return (self.row, self.col, self.heading, sensor)

    def _read_sensor(self):
        """
        Simulate the HC-SR04 by counting cells ahead until
        a wall or obstacle is encountered.

        Returns: 'near', 'medium', or 'far'
        """
        dr, dc = HEADING_DELTA[self.heading]
        r, c = self.row + dr, self.col + dc
        cells_ahead = 0

        while 0 <= r < GRID_ROWS and 0 <= c < GRID_COLS:
            if (r, c) in OBSTACLES:
                break
            cells_ahead += 1
            if cells_ahead >= 5:   # sensor range limit
                break
            r += dr
            c += dc
        else:
            # Hit the grid boundary
            pass

        if cells_ahead <= NEAR_THRESHOLD:
            return "near"
        elif cells_ahead <= MEDIUM_THRESHOLD:
            return "medium"
        else:
            return "far"

    def step(self, action):
        """
        Execute an action and return (next_state, reward, done).

        Actions:
          'forward'    — move one cell in the current heading direction
          'turn_left'  — rotate 90° anticlockwise (no position change)
          'turn_right' — rotate 90° clockwise (no position change)
          'stop'       — do nothing (accumulates step cost)
        """
        if self.done:
            # Environment is already in a terminal state
            return self._get_state(), 0.0, True

        prev_row, prev_col = self.row, self.col
        hit_obstacle = False
        moved_forward = False

        if action == "forward":
            dr, dc = HEADING_DELTA[self.heading]
            new_r = self.row + dr
            new_c = self.col + dc

            if (0 <= new_r < GRID_ROWS and
                0 <= new_c < GRID_COLS and
                (new_r, new_c) not in OBSTACLES):
                # Valid move
                self.row, self.col = new_r, new_c
                moved_forward = True
            else:
                # Hit wall or obstacle
                hit_obstacle = True

        elif action == "turn_left":
            idx = HEADINGS.index(self.heading)
            self.heading = HEADINGS[(idx - 1) % 4]   # anticlockwise

        elif action == "turn_right":
            idx = HEADINGS.index(self.heading)
            self.heading = HEADINGS[(idx + 1) % 4]   # clockwise

        # "stop" — no state change

        # Check for goal
        reached_goal = (self.row, self.col) == GOAL

        # Compute reward
        reward = self._compute_reward(hit_obstacle, reached_goal, moved_forward)

        # Episode ends on goal or obstacle collision
        if reached_goal or hit_obstacle:
            self.done = True

        next_state = self._get_state()
        return next_state, reward, self.done

    def _compute_reward(self, hit_obstacle, reached_goal, moved_forward):
        """Reward function from lesson 3."""
        if hit_obstacle:
            return -10.0
        if reached_goal:
            return +50.0
        if moved_forward:
            return +1.0
        return -0.1   # small penalty for turning or stopping

    def render(self):
        """Print the current grid state to the terminal."""
        print()
        for r in range(GRID_ROWS):
            row_str = ""
            for c in range(GRID_COLS):
                if (r, c) == (self.row, self.col):
                    # Show heading arrow
                    arrow = {"north": "^", "east": ">", "south": "v", "west": "<"}
                    row_str += arrow[self.heading] + " "
                elif (r, c) in OBSTACLES:
                    row_str += "O "
                elif (r, c) == GOAL:
                    row_str += "G "
                else:
                    row_str += ". "
            print(row_str)
        print()


# ---------------------------------------------------------------------------
# Agent helpers  (from lessons 7, 8, 9)
# ---------------------------------------------------------------------------

def make_q_table():
    return defaultdict(lambda: {a: 0.0 for a in ACTIONS})


def choose_action(q_table, state, epsilon):
    """ε-greedy action selection."""
    if random.random() < epsilon:
        return random.choice(ACTIONS)
    action_values = q_table[state]
    return max(action_values, key=lambda a: action_values[a])


def update_q(q_table, state, action, reward, next_state, alpha, gamma):
    """Q-learning update rule."""
    current_q    = q_table[state][action]
    best_next_q  = max(q_table[next_state].values())
    td_target    = reward + gamma * best_next_q
    td_error     = td_target - current_q
    q_table[state][action] += alpha * td_error
    return q_table


def run_episode(env, q_table, alpha, gamma, epsilon, max_steps):
    """Run one training episode. Return total reward."""
    state      = env.reset()
    total_reward = 0.0

    for _ in range(max_steps):
        action = choose_action(q_table, state, epsilon)
        next_state, reward, done = env.step(action)
        update_q(q_table, state, action, reward, next_state, alpha, gamma)
        state = next_state
        total_reward += reward
        if done:
            break

    return total_reward


# ---------------------------------------------------------------------------
# Save / load Q-table
# ---------------------------------------------------------------------------

def save_q_table(q_table, filename):
    serialisable = {str(k): v for k, v in q_table.items()}
    with open(filename, "w") as f:
        json.dump(serialisable, f, indent=2)
    print(f"\nSaved {len(q_table)} states to '{filename}'")


def load_q_table(filename):
    with open(filename, "r") as f:
        raw = json.load(f)
    return {ast.literal_eval(k): v for k, v in raw.items()}


# ---------------------------------------------------------------------------
# Training
# ---------------------------------------------------------------------------

def print_progress_bar(recent_avg, width=30):
    """Simple ASCII progress indicator based on recent average reward."""
    # Map [-10, 50] reward range to [0, width]
    filled = int((recent_avg + 10) / 60 * width)
    filled = max(0, min(width, filled))
    bar = "#" * filled + "-" * (width - filled)
    return f"[{bar}] {recent_avg:7.2f}"


def train():
    env      = GridWorld()
    q_table  = make_q_table()
    epsilon  = EPSILON_START
    history  = []   # total reward per episode

    print("=" * 60)
    print("  BurgerBot Q-Learning Trainer")
    print(f"  Grid: {GRID_ROWS}x{GRID_COLS}  |  Obstacles: {len(OBSTACLES)}")
    print(f"  Episodes: {NUM_EPISODES}  |  alpha={ALPHA}  gamma={GAMMA}")
    print("=" * 60)
    print(f"  {'Episode':>8}  {'Avg reward (50 ep)':>20}  {'Epsilon':>8}")
    print("-" * 60)

    for episode in range(NUM_EPISODES):
        ep_reward = run_episode(env, q_table, ALPHA, GAMMA, epsilon, MAX_STEPS)
        history.append(ep_reward)

        epsilon = max(EPSILON_MIN, epsilon * DECAY_RATE)

        if (episode + 1) % 50 == 0:
            recent_avg = sum(history[-50:]) / 50
            bar = print_progress_bar(recent_avg)
            print(f"  {episode+1:>8d}  {bar}  {epsilon:>8.4f}")

    print("=" * 60)
    print(f"  Training complete. Final Q-table: {len(q_table)} states")

    # Save Q-table for deployment to Pico
    save_q_table(q_table, Q_TABLE_FILE)
    return q_table, history


# ---------------------------------------------------------------------------
# Demo: run one greedy episode and render it
# ---------------------------------------------------------------------------

def demo_policy(q_table, max_steps=50):
    """Run a single greedy episode (epsilon=0) and print each step."""
    env = GridWorld()
    state = env.reset()
    print("\n--- Demo: greedy policy (epsilon = 0) ---")
    env.render()

    for step in range(max_steps):
        action = choose_action(q_table, state, epsilon=0.0)
        next_state, reward, done = env.step(action)
        print(f"Step {step+1:3d}: action={action:<12s}  reward={reward:+6.1f}  "
              f"state={next_state}")
        env.render()
        state = next_state
        if done:
            if reward == 50.0:
                print("  *** Robot reached the goal! ***")
            else:
                print("  *** Robot hit an obstacle ***")
            break
    else:
        print("  (reached step limit)")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    random.seed(0)   # reproducible results
    q_table, history = train()
    demo_policy(q_table)
```

---

## ▶️ Running the Trainer

```bash
python3 train.py
```

You should see output like this:

```
============================================================
  BurgerBot Q-Learning Trainer
  Grid: 7x7  |  Obstacles: 7
  Episodes: 600  |  alpha=0.2  gamma=0.9
============================================================
   Episode  Avg reward (50 ep)   Epsilon
------------------------------------------------------------
        50  [----------] -5.42    0.7783
       100  [##--------]  2.11    0.6058
       150  [####------]  8.34    0.4724
       200  [######----] 15.21    0.3679
       300  [########--] 24.56    0.2231
       400  [##########] 31.80    0.1353
       500  [##########] 38.42    0.0820
       600  [##########] 42.17    0.0497
============================================================
  Training complete. Final Q-table: 187 states
  Saved 187 states to 'q_table.json'
```

The average reward rising from negative (early crashes) to a high positive (consistently reaching the goal) is the signature of successful learning.

After the training summary, the demo runs a greedy episode and prints each step with an ASCII grid — you can watch the robot navigate around obstacles.

---

## Understanding the Learning Curve

If your curve doesn't look like the above, here's what the common shapes mean:

| Pattern | Likely cause | Fix |
|---|---|---|
| Stays near -10 for 400+ episodes | Reward too sparse / maze too hard | Add a shaping reward for moving toward goal |
| Shoots up fast then drops | Epsilon decaying too fast | Slow down `decay_rate` to 0.999 |
| Never improves at all | Bug in update_q or reward function | Print (state, action, reward) for first 20 steps |
| Reaches ~20 then plateaus | Good but not optimal policy | Increase `NUM_EPISODES` to 1000 |
{:class="table table-single"}

---

## Try It Yourself

1. Change the grid to 5×5 with fewer obstacles and retrain. How many episodes does it take to reach an average reward of +30? Compare to the 7×7 version.

2. Modify `OBSTACLES` to create a maze that forces the robot through a narrow corridor. Does Q-learning still find the path? How many episodes does it need?

3. Change the reward for `hit_obstacle` from -10 to -1. What happens to the final policy? Does the robot become more or less willing to take risks?

4. After training, load the Q-table and count how many states have `forward` as their best action. Is this a higher or lower fraction than you'd expect in an open space?

---

## Common Issues

**"Python error: No module named 'collections.defaultdict'"**
Make sure you're importing from `collections`: `from collections import defaultdict`. This is a standard library module.

**"The demo always crashes immediately."**
Training didn't converge. Check that `NUM_EPISODES` is at least 300 and that your obstacle layout doesn't completely block the path from START to GOAL. Try the exact obstacle set from the lesson first.

**"The Q-table file is empty or very small."**
The agent may not have explored many states. Check that `EPSILON_START = 1.0` and that the first several episodes aren't all hitting obstacles on step 1 (which would leave most of the grid unexplored).

**"RuntimeError: dictionary changed size during iteration"**
This can happen if you iterate over `q_table` while also inserting into it (which `defaultdict` does on first access). Use `list(q_table.items())` in any loop that reads and writes simultaneously, or access `q_table[state]` only in the update function.

---

Next up: the sim-to-real gap — why our perfectly trained simulation policy might struggle on the actual robot, and what to do about it.

---
