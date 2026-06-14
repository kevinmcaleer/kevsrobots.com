---
layout: lesson
title: Train, Evaluate, and Export
author: Kevin McAleer
type: page
cover: assets/cover.jpg
date: 2026-06-14
previous: 12_meet_stable_baselines3.html
next: 14_summary.html
description: "The capstone lesson \u2014 train PPO on BurgerBotEnv, evaluate the policy,\
  \ save and load the model, and export a lookup table that runs on a Pico."
percent: 84
duration: 7
date_updated: 2026-06-14
navigation:
- name: Reinforcement Learning with Gymnasium
- content:
  - section: Introduction
    content:
    - name: Reinforcement Learning with Gymnasium
      link: 00_intro.html
  - section: Why Gymnasium
    content:
    - name: From Hand-Built to Standard
      link: 01_from_handbuilt_to_standard.html
    - name: Your First Gymnasium Environment
      link: 02_first_environment.html
    - name: Terminated vs Truncated
      link: 03_terminated_vs_truncated.html
  - section: The Gymnasium API in Depth
    content:
    - name: Observation Spaces and Action Spaces
      link: 04_spaces.html
    - name: Q-Learning on Gymnasium
      link: 05_qlearning_on_gymnasium.html
    - name: Seeding and the Episode Lifecycle
      link: 06_seeding_and_lifecycle.html
  - section: Building Your Own Environment
    content:
    - name: Anatomy of a Custom Environment
      link: 07_anatomy_of_custom_env.html
    - name: Wrapping the BurgerBot
      link: 08_wrapping_the_burgerbot.html
    - name: Registering and Rendering
      link: 09_registering_and_rendering.html
    - name: Gymnasium Wrappers
      link: 10_wrappers.html
  - section: Scaling Up to Neural Networks
    content:
    - name: Vectorised Environments
      link: 11_vectorized_envs.html
    - name: Meet Stable-Baselines3
      link: 12_meet_stable_baselines3.html
    - name: Train, Evaluate, and Export
      link: 13_train_evaluate_export.html
    - name: "What You Have Built \u2014 and Where to Go Next"
      link: 14_summary.html
---


![Cover](assets/cover.jpg){:class="cover"}

---

## Bringing It All Together

This is the capstone. In the previous course you trained a Q-table on the BurgerBot grid-world and saved it as `q_table.json` so a Pico could load and execute the policy. In this lesson you will do the same thing — but the policy comes from a neural network trained with Stable-Baselines3, not a hand-written Q-learning loop.

The closing loop is this: neural networks cannot run on a Pico, but BurgerBot's observation space is small enough that we can extract a lookup table from the trained network by querying it over every possible state. The result is a JSON file in the same format as the previous course's `q_table.json` — and the same Pico loader from lesson 13 of that course can read it directly.

## The Complete Training Script

Save this as `train_burgerbot.py`. It covers environment setup, training, saving, and loading.

```python
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.env_util import make_vec_env

# ── BurgerBotEnv definition ────────────────────────────────────────────────
# (Paste the full BurgerBotEnv class here — identical to lesson 08)

GRID_ROWS, GRID_COLS = 7, 7
OBSTACLES = {(0,3),(1,1),(1,5),(3,0),(3,3),(4,5),(5,1)}
START, GOAL = (0,0), (6,6)
ACTIONS   = ["forward","turn_left","turn_right","stop"]
HEADINGS  = ["north","east","south","west"]
HEADING_DELTA = {"north":(-1,0),"east":(0,1),"south":(1,0),"west":(0,-1)}
NEAR, MEDIUM = 1, 3

class BurgerBotEnv(gym.Env):
    metadata = {"render_modes": ["ansi"]}
    def __init__(self, max_steps=150, render_mode=None):
        super().__init__()
        self.max_steps = max_steps
        self.render_mode = render_mode
        self.observation_space = spaces.MultiDiscrete([GRID_ROWS, GRID_COLS, 4, 3])
        self.action_space = spaces.Discrete(4)
    def _sensor(self):
        dr,dc = HEADING_DELTA[self.heading]
        r,c = self.row+dr, self.col+dc
        n=0
        while 0<=r<GRID_ROWS and 0<=c<GRID_COLS:
            if (r,c) in OBSTACLES: break
            n+=1
            if n>=5: break
            r+=dr; c+=dc
        if n<=NEAR: return 0
        if n<=MEDIUM: return 1
        return 2
    def _obs(self):
        return np.array([self.row,self.col,HEADINGS.index(self.heading),self._sensor()],dtype=np.int64)
    def reset(self,*,seed=None,options=None):
        super().reset(seed=seed)
        self.row,self.col=START; self.heading="east"; self.steps=0
        return self._obs(),{}
    def step(self,action):
        self.steps+=1
        a=ACTIONS[action]; hit=False; moved=False
        if a=="forward":
            dr,dc=HEADING_DELTA[self.heading]; nr,nc=self.row+dr,self.col+dc
            if 0<=nr<GRID_ROWS and 0<=nc<GRID_COLS and (nr,nc) not in OBSTACLES:
                self.row,self.col=nr,nc; moved=True
            else: hit=True
        elif a=="turn_left":
            self.heading=HEADINGS[(HEADINGS.index(self.heading)-1)%4]
        elif a=="turn_right":
            self.heading=HEADINGS[(HEADINGS.index(self.heading)+1)%4]
        reached=(self.row,self.col)==GOAL
        if hit: reward=-10.0
        elif reached: reward=50.0
        elif moved: reward=1.0
        else: reward=-0.1
        terminated=reached or hit; truncated=self.steps>=self.max_steps
        return self._obs(),reward,terminated,truncated,{}
    def render(self):
        if self.render_mode!="ansi": return
        h={"north":"^","east":">","south":"v","west":"<"}
        out=""
        for r in range(GRID_ROWS):
            for c in range(GRID_COLS):
                if (r,c)==(self.row,self.col): out+=h[self.heading]+" "
                elif (r,c) in OBSTACLES: out+="O "
                elif (r,c)==GOAL: out+="G "
                else: out+=". "
            out+="\n"
        return out

# ── Training ──────────────────────────────────────────────────────────────────
print("Creating environment...")
env = BurgerBotEnv()

print("Creating PPO model...")
model = PPO(
    "MlpPolicy",
    env,
    verbose=0,
    learning_rate=3e-4,
    n_steps=512,
    batch_size=64,
    gamma=0.99,
    seed=42,
)

print("Training... (this takes about 1-2 minutes on CPU)")
model.learn(total_timesteps=50000)
print("Training complete.")

# ── Save ─────────────────────────────────────────────────────────────────────
model.save("burgerbot_ppo")
print("Model saved to burgerbot_ppo.zip")

# ── Load ─────────────────────────────────────────────────────────────────────
loaded_model = PPO.load("burgerbot_ppo", env=env)
print("Model loaded from burgerbot_ppo.zip")

# ── Evaluate ──────────────────────────────────────────────────────────────────
mean_reward, std_reward = evaluate_policy(
    loaded_model,
    env,
    n_eval_episodes=20,
    deterministic=True,
)
print(f"\nEvaluation over 20 episodes:")
print(f"  Mean reward: {mean_reward:.1f} +/- {std_reward:.1f}")

# Also count goal reach rate manually
successes = 0
for ep in range(20):
    obs, info = env.reset(seed=1000 + ep)
    done = False
    while not done:
        action, _ = loaded_model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)
        done = terminated or truncated
        if reward == 50.0:
            successes += 1

print(f"  Goal reach rate: {successes}/20 ({100*successes//20}%)")

env.close()
```

## Expected Training Results

With 50,000 timesteps, PPO on BurgerBot typically reaches:

- Mean reward: 35 to 50 per episode
- Goal reach rate: 60 to 90 % (the variance is higher with a random seed)

Increase `total_timesteps` to 200,000 for a more reliably trained policy. The training takes about 5-10 minutes on CPU.

## Exporting the Policy as a Lookup Table

Here is where this course closes the loop with the previous one. BurgerBot's observation space has 588 possible states. We can query the trained neural network on all 588 and record the best action — producing a lookup table identical in format to the Q-table from the previous course.

```python
import json

def export_policy_to_json(model, filename="nn_policy.json"):
    """
    Query the trained neural network for every possible BurgerBot state
    and save the best action as a JSON lookup table.

    The format matches q_table.json from the RL course so the Pico
    loader from lesson 13 of that course can read this file directly.
    """
    policy_table = {}

    for row in range(GRID_ROWS):
        for col in range(GRID_COLS):
            for heading_idx in range(4):
                for sensor in range(3):
                    # Build the observation numpy array
                    obs = np.array([row, col, heading_idx, sensor], dtype=np.int64)

                    # Query the neural network
                    action, _ = model.predict(obs, deterministic=True)
                    action = int(action)

                    # Key format: "row,col,heading,sensor" — matches RL course
                    key = f"{row},{col},{heading_idx},{sensor}"

                    # Store both the best action and dummy Q-values
                    # so the Pico loader (which expects a dict of action->value)
                    # can read the file unchanged.
                    policy_table[key] = {
                        a: (1.0 if i == action else 0.0)
                        for i, a in enumerate(ACTIONS)
                    }

    with open(filename, "w") as f:
        json.dump(policy_table, f, indent=2)

    print(f"Exported policy for {len(policy_table)} states to {filename}")
    return policy_table

# Export the loaded model's policy
policy = export_policy_to_json(loaded_model)

# Spot-check: what does the agent do at the starting position?
start_key = "0,0,1,2"   # row=0, col=0, heading=east(1), sensor=far(2)
print(f"\nAt start state {start_key}:")
print(f"  Action values: {policy[start_key]}")
best = max(policy[start_key], key=policy[start_key].get)
print(f"  Best action:   {best} (index {ACTIONS.index(best)})")
```

## How the Pico Reads This File

The Pico loader from the previous course (lesson 13) reads `q_table.json` and picks the action with the highest value for the current state:

```python
# Pico loader (from RL course lesson 13) — unchanged:
import json

with open("q_table.json") as f:
    q_table = json.load(f)

def choose_action(state):
    key = f"{state[0]},{state[1]},{state[2]},{state[3]}"
    if key in q_table:
        return max(q_table[key], key=q_table[key].get)
    return "forward"   # fallback if state not seen
```

Our exported `nn_policy.json` has the same structure. The best action has value `1.0` and all others have `0.0`, so `max(...)` correctly returns the best action. You can drop `nn_policy.json` in place of `q_table.json` on the Pico and the loader needs no changes.

## Getting Neural Networks onto Hardware

Two realistic deployment paths:

**Path A: Policy table (works for BurgerBot)**

This is what we just did. It works because BurgerBot's observation space is small and fully discrete — we can enumerate all 588 states. The Pico runs no neural network; it just does a dictionary lookup.

This approach works any time your observation space is small and discrete. For a real robot with a camera or continuous sensors, it does not scale.

**Path B: Inference on a Raspberry Pi**

For bigger observation spaces (camera images, multiple sensors), the deployment path is to run inference on a Raspberry Pi rather than a Pico. A Pi 4 or Pi 5 can run PyTorch in CPU mode. You would:

1. Save the SB3 model with `model.save("policy")`
2. Load it on the Pi: `model = PPO.load("policy")`
3. Read sensor data on the Pi, call `model.predict(obs)`, and send the action to the robot (over serial, I2C, or GPIO)

The Pico handles hardware I/O at the bottom; the Pi handles neural network inference in the middle.

## Try It Yourself

1. Re-run the training with `total_timesteps=5000` (very short). Export the policy. Does the exported table still beat a random agent? Use the manual evaluation loop to measure the success rate.
2. Open `nn_policy.json` in a text editor and find the entry for the goal state `"6,6,..."`. What action does the network choose when the robot is already at the goal? Does this make sense?
3. Compare the exported `nn_policy.json` to the `q_table.json` from your previous course. Are the chosen actions the same for the starting state? If they differ, which policy reaches the goal in fewer steps?

## Common Issues

**Problem:** `ValueError: observation is not in observation space` during `model.predict`
**Solution:** Make sure `obs` has `dtype=np.int64`. `model.predict` checks the dtype against the declared observation space.
**Why:** SB3 respects Gymnasium's space declarations and validates observations before the forward pass.

**Problem:** The policy reaches the goal 0% of the time after 50,000 timesteps.
**Solution:** Check that the reward function is correct. A `reward=-10` on every step suggests the environment is hitting the obstacle logic on all forward moves. Print `env.unwrapped.row, env.unwrapped.col` during a rollout to debug position.
**Why:** PPO converges reliably on BurgerBot with 50k steps. If it does not, there is almost certainly a bug in the environment or reward function.

**Problem:** `json.JSONDecodeError` when the Pico loads the exported file.
**Solution:** NumPy integers are not JSON-serialisable by default. Make sure you convert with `int(action)` before storing in the dict. The code above does this, but if you modify it, watch out for NumPy types slipping in.
**Why:** Python's `json.dump` cannot serialise `numpy.int64`. Use `int()` to cast before storing.
