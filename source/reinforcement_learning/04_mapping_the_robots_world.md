---
title: Mapping the Robot's World
description: Turn continuous ultrasonic sensor readings into discrete states a Q-table can index — and understand the trade-offs in state-space size.
layout: lesson
type: page
cover: assets/cover.jpg
date_updated: 2026-06-13
---

![Cover](assets/cover.jpg){:class="cover"}

---

Sensors lie, in a way. Not maliciously — they tell you what they measure. But what they measure is a continuous number (distance in centimetres, voltage, angle in degrees), and a tabular Q-table needs a *discrete* state label it can use as a dictionary key.

This lesson is about building the bridge between the messy analogue world and the clean world of a lookup table.

---

## The Problem with Raw Sensor Values

The HC-SR04 can return any value from about 2 cm to 400 cm. If we used the raw reading as a state, we'd need a Q-table row for every possible centimetre value — up to 400 separate rows, each of which the agent would need to visit many times during training before learning anything useful about it.

Worse, readings that are practically identical (37 cm vs 38 cm) would be treated as completely different situations, even though the correct action is almost certainly the same in both cases.

The solution is **discretisation**: grouping nearby sensor values into labelled bins.

---

## Building a Discretise Function

Here's the core idea: instead of 400 possible distances, we use just three — **near**, **medium**, and **far**. The agent only needs to learn a good action for each of those three situations.

```python
def discretize(distance_cm):
    """
    Map a continuous HC-SR04 reading into one of three distance bands.

    Args:
        distance_cm (float): raw sensor reading in centimetres

    Returns:
        str: one of 'near', 'medium', 'far'

    Thresholds are chosen to give the robot time to react:
    - 'near'  : obstacle within braking distance (~15 cm at typical speed)
    - 'medium': obstacle visible but stoppable (15–40 cm)
    - 'far'   : clear path, safe to move forward (> 40 cm)
    """
    if distance_cm < 15:
        return "near"
    elif distance_cm < 40:
        return "medium"
    else:
        return "far"
```

Let's verify it with a few test cases:

```python
# Quick sanity check
test_distances = [5, 14.9, 15, 25, 39.9, 40, 100, 300]
for d in test_distances:
    print(f"{d:6.1f} cm  ->  {discretize(d)}")

# Expected output:
#    5.0 cm  ->  near
#   14.9 cm  ->  near
#   15.0 cm  ->  medium
#   25.0 cm  ->  medium
#   39.9 cm  ->  medium
#   40.0 cm  ->  far
#  100.0 cm  ->  far
#  300.0 cm  ->  far
```

---

## States in the Grid World

For our simulation, the robot lives on a 2-D grid. Its state is its (row, column) position on the grid plus what's directly in front of it. We combine these into a single tuple:

```python
# Grid position + sensor reading = full state
state = (row, col, distance_band)

# Example:
state = (2, 3, "near")   # at grid cell (2,3), obstacle very close ahead
state = (0, 0, "far")    # at top-left corner, clear ahead
```

Using a tuple as a dictionary key works perfectly in Python:

```python
q_table = {}
q_table[(2, 3, "near")] = {"forward": -2.1, "turn_left": 0.4, "turn_right": 0.3, "stop": 0.0}
q_table[(0, 0, "far")]  = {"forward":  3.7, "turn_left": 0.1, "turn_right": 0.2, "stop": -0.1}
```

The Q-table grows one row per unique (row, col, distance_band) combination ever encountered during training. Unvisited states simply don't appear in the dictionary — we'll handle that with a `defaultdict` in lesson 7.

---

## State-Space Size Trade-offs

The number of bins you choose has a big effect on how quickly the agent can learn.

**Too few bins (e.g. only "obstacle" or "no obstacle"):**
- Small state space → the agent visits every state many times → learns quickly
- But the agent loses important information. "Near" and "medium" require different responses, and coarse binning can't distinguish them
- Results in a policy that can seem jerky or unsafe

**Too many bins (e.g. 20 distance bands):**
- Large state space → the agent visits each state less often → learns slowly
- Fine-grained distinctions might not improve the policy at all (turning at 37 cm vs 38 cm is the same decision)
- Training takes much longer and requires many more episodes

For our BurgerBot task, three distance bands (near / medium / far) is a good starting point. You might experiment with four or five, but the improvement often isn't worth the longer training time.

---

## Combining Multiple Sensor Dimensions

Real robots have more than one thing to sense. If our robot also tracked heading (which direction it's facing on the grid), we'd add that to the state:

```python
HEADINGS = ["north", "east", "south", "west"]
DISTANCE_BANDS = ["near", "medium", "far"]

state = (row, col, heading, distance_band)

# Total possible states = grid_rows * grid_cols * 4 headings * 3 bands
# On a 5x5 grid: 5 * 5 * 4 * 3 = 300 states
# Very manageable for a lookup table!
```

Compare that to using raw sensor data. A 16-bit distance reading plus a continuous heading in tenths of a degree gives 65,536 × 3,600 = 235,929,600 possible states. A lookup table with that many rows would use gigabytes of memory and take millions of training episodes to fill.

Discretisation is not a concession or a workaround — it is the correct design choice for tabular RL.

---

## Try It Yourself

1. Modify `discretize()` to use four bands: "very_near" (< 8 cm), "near" (8–20 cm), "medium" (20–45 cm), and "far" (> 45 cm). Write a test loop that verifies boundary values.

2. Calculate the total number of states for a 6×6 grid with 4 headings and your new 4-band distance discretisation. Is this still manageable for a lookup table?

3. Think about a robot arm with two joints, each measured in degrees (0–180). How would you discretise the joint angles? What granularity would you choose, and why?

---

## Common Issues

**"My sensor returns negative values or very large numbers sometimes."**
The HC-SR04 occasionally returns bogus readings (timeout values, reflections from unusual surfaces). In simulation this doesn't happen, but on real hardware you should clamp and filter the reading before discretising. We cover this in lesson 12.

**"Should I include the robot's action history in the state?"**
Not for this project. Adding history (e.g. "I turned left last time") makes the state space much larger and training much slower. The Markov property (next lesson) gives us a principled reason why the current sensor reading is usually enough.

**"What if my robot is at the edge of the grid and there's no obstacle?"**
In the simulation, we'll treat grid boundaries as obstacles — any move that would take the robot off the grid results in a collision penalty and no position change. This simplifies the state representation because we don't need special edge-case state labels.

---

Next up: why "the present state is all you need" — and the Markov property explained in plain English.

---
