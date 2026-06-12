---
title: The Q-Table
description: Build the robot's cheat sheet — a Python dictionary that stores the estimated value of every action in every state.
layout: lesson
type: page
cover: assets/cover.jpg
date_updated: 2026-06-13
---

![Cover](assets/cover.jpg){:class="cover"}

---

We've talked about learning from rewards and discounting the future. Now we need a data structure to hold everything the robot has learned so far. That structure is called the **Q-table**, and it's simpler than its name suggests.

---

## What Is a Q-Table?

Q stands for **quality** (sometimes called **action-value**). The Q-table is a lookup table with one row per state and one column per action. Each cell holds a single number: the agent's best current estimate of how much total (discounted) reward it will earn if it takes that action in that state and then follows its best policy from there onward.

```
State               | forward | turn_left | turn_right | stop
--------------------|---------|-----------|------------|------
(1,0,"east","far")  |   3.21  |    0.42   |    0.38    | -0.10
(1,1,"east","near") |  -4.50  |    1.21   |    1.19    |  0.05
(1,2,"north","far") |   2.80  |    0.10   |    0.34    | -0.10
...
```

The **best action** in any state is simply the column with the highest value in that row. The agent can make a decision just by looking up its current state and picking the action with the biggest Q-value.

---

## Implementing the Q-Table in Python

We'll use a Python `defaultdict` to store the Q-table. It automatically creates a zero-initialised action dictionary whenever the agent encounters a new state for the first time.

```python
from collections import defaultdict

# Actions our BurgerBot can take
ACTIONS = ["forward", "turn_left", "turn_right", "stop"]

def make_q_table():
    """
    Create an empty Q-table as a defaultdict.

    When the agent visits a new state for the first time,
    the defaultdict creates an entry automatically with Q=0.0
    for every action. This avoids KeyError and means we don't
    need to pre-populate all states upfront.
    """
    return defaultdict(lambda: {action: 0.0 for action in ACTIONS})

q_table = make_q_table()

# Accessing a brand new state works immediately
print(q_table[(0, 0, "north", "far")])
# Output: {'forward': 0.0, 'turn_left': 0.0, 'turn_right': 0.0, 'stop': 0.0}
```

---

## Reading the Best Action

Given a Q-table and a state, picking the best action is a one-liner:

```python
def best_action(q_table, state):
    """
    Return the action with the highest Q-value in this state.

    If multiple actions tie, max() returns the first one found
    (dictionary insertion order in Python 3.7+).
    """
    action_values = q_table[state]
    return max(action_values, key=lambda a: action_values[a])

# Example — imagine the Q-table already has these values:
q_table[(1, 1, "east", "near")] = {
    "forward":    -4.50,
    "turn_left":   1.21,
    "turn_right":  1.19,
    "stop":        0.05,
}

print(best_action(q_table, (1, 1, "east", "near")))
# Output: 'turn_left'   (highest Q-value = 1.21)
```

---

## Visualising a Small Q-Table

It's helpful to print your Q-table in a readable format during training so you can watch values evolve. Here's a simple printer for our 4-action case:

```python
def print_q_table(q_table, max_rows=10):
    """
    Print the Q-table in a readable grid format.
    Only shows states the agent has actually visited.
    """
    header = f"{'State':<35} {'forward':>8} {'turn_left':>10} {'turn_right':>11} {'stop':>7}"
    print(header)
    print("-" * len(header))

    for i, (state, values) in enumerate(q_table.items()):
        if i >= max_rows:
            print(f"  ... ({len(q_table) - max_rows} more rows)")
            break
        row = (
            f"{str(state):<35}"
            f"{values['forward']:>8.2f}"
            f"{values['turn_left']:>10.2f}"
            f"{values['turn_right']:>11.2f}"
            f"{values['stop']:>7.2f}"
        )
        print(row)
```

After 50 training episodes, a call to `print_q_table(q_table)` might give you something like:

```
State                               forward  turn_left  turn_right     stop
------------------------------------------------------------------------
(0, 0, 'north', 'far')                1.23       0.21        0.19    -0.10
(0, 1, 'east', 'far')                 2.11       0.15        0.14    -0.10
(1, 1, 'east', 'near')               -3.10       1.40        1.38     0.02
...
```

You can see the agent has already learned that going forward when an obstacle is near (-3.10) is a bad idea, while turning is relatively safe.

---

## Saving and Loading the Q-Table as JSON

When training finishes, we save the Q-table to disk so it can be loaded by the Pico. The tricky part is that dictionary keys are tuples (not valid JSON keys), so we convert them to strings.

```python
import json

def save_q_table(q_table, filename="q_table.json"):
    """
    Save Q-table to a JSON file.
    Tuple keys (e.g. (1, 2, 'east', 'near')) are serialised as strings.
    """
    serialisable = {str(k): v for k, v in q_table.items()}
    with open(filename, "w") as f:
        json.dump(serialisable, f, indent=2)
    print(f"Saved {len(q_table)} states to {filename}")

def load_q_table(filename="q_table.json"):
    """
    Load Q-table from a JSON file.
    String keys are converted back to tuples using eval().
    """
    import ast
    with open(filename, "r") as f:
        raw = json.load(f)
    q_table = {ast.literal_eval(k): v for k, v in raw.items()}
    return q_table

# Usage after training:
save_q_table(q_table, "q_table.json")   # on the laptop
# ... copy q_table.json to the Pico ...
loaded = load_q_table("q_table.json")   # on the laptop to verify
```

> **Note:** `ast.literal_eval()` safely converts a string representation of a Python tuple back into an actual tuple. We use it instead of `eval()` because it rejects anything that isn't a valid Python literal — so it's safe to use on data you loaded from a file.

---

## Try It Yourself

1. Create a Q-table using `make_q_table()` and manually insert Q-values for five different states. Call `best_action()` for each and verify the results make sense.

2. Extend `print_q_table()` to highlight the best action in each row (e.g. surround it with `>>` markers). Verify it correctly identifies the highest value.

3. Write a function `q_table_size_kb(q_table)` that estimates the memory footprint of the Q-table in kilobytes (assume each float is 8 bytes, each string key averages 20 bytes). How big would the table be for a fully explored 5×5 grid with 4 headings and 3 distance bands?

---

## Common Issues

**"My Q-table keeps growing without bound."**
This is normal during training — each new state visited adds a row. On a bounded grid with finite state space, it stabilises once all states have been visited. If it keeps growing on a fixed grid, check that your state representation is deterministic and consistent.

**"Q-values are drifting to very large positive or negative numbers."**
This can happen with an untuned learning rate (lesson 9) or if reward values are very large. Keep reward magnitudes moderate (e.g. ±10 to ±50) and your learning rate between 0.01 and 0.5.

**"json.dump fails with 'keys must be strings'."**
This is because Python JSON requires string keys. Use `{str(k): v ...}` in `save_q_table()` as shown above. Loading uses `ast.literal_eval()` to convert them back.

---

Next up: ε-greedy exploration — the strategy that decides when the robot should trust its Q-table and when it should try something random.

---
