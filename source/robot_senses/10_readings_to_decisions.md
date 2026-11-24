---
title: From Readings to Decisions
description: Turn sensor numbers into robot behaviour with thresholds, hysteresis to stop jitter, and a simple state machine
layout: lesson
type: page
cover: /learn/robot_senses/assets/banners/10_readings_to_decisions.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

You can now read all five senses. Next is the part that turns a bag of sensors into a robot: deciding what the numbers mean. This is the "think" in sense, think, act. All the code in this lesson runs anywhere, with made-up readings, so you can experiment before the robot is even charged.

## Thresholds

The simplest decision is a threshold: if a number crosses a line, do something. Kev's robot has a stop distance of 10 cm. Closer than that, it stops.

```python
STOP_DISTANCE_CM = 10

def should_stop(distance):
    # None means nothing in range, so there is nothing to stop for
    if distance is None:
        return False
    return distance < STOP_DISTANCE_CM

for d in (None, 50, 12, 9.5, 4):
    print(d, "->", "STOP" if should_stop(d) else "go")
```

That works. But watch what happens with a reading that hovers around the line.

## The flicker problem

A real sensor is a little noisy. Say the robot is 10 cm from a wall and the readings wobble between 9.8 and 10.2. With one threshold, the robot flips between stop and go every loop. It stutters, wastes power and sounds awful if the buzzer is involved.

```python
STOP_DISTANCE_CM = 10

# A robot sitting right on the line, with noisy readings
noisy = [10.2, 9.8, 10.1, 9.9, 10.3, 9.7]

state = "go"
changes = 0
for d in noisy:
    new_state = "stop" if d < STOP_DISTANCE_CM else "go"
    if new_state != state:
        changes += 1
        state = new_state
    print(d, state)
print("State changed", changes, "times in", len(noisy), "readings")
```

## Hysteresis: two lines, not one

The fix is to use **two** thresholds. Stop when the robot gets closer than 10 cm, but don't go again until it's farther than 20 cm. The gap between them is a dead zone, and it stops the flicker. Kev's `config.py` has both values: `STOP_DISTANCE_CM = 10` and `CLEAR_DISTANCE_CM = 20`.

```python
STOP_DISTANCE_CM = 10
CLEAR_DISTANCE_CM = 20

class ObstacleGate:
    """Remembers whether the way is blocked, with a dead zone."""

    def __init__(self):
        self.blocked = False

    def update(self, distance):
        if distance is None:
            self.blocked = False          # nothing in range
        elif not self.blocked and distance < STOP_DISTANCE_CM:
            self.blocked = True
        elif self.blocked and distance > CLEAR_DISTANCE_CM:
            self.blocked = False
        return self.blocked

gate = ObstacleGate()
changes = 0
last = False
for d in [10.2, 9.8, 10.1, 9.9, 10.3, 9.7, 15, 25, 30]:
    now = gate.update(d)
    if now != last:
        changes += 1
        last = now
    print(d, "blocked" if now else "clear")
print("State changed", changes, "times")
```

Run it. The state changes once to blocked and once back to clear. The same noisy readings no longer cause a stutter.

**Why this matters:** hysteresis is everywhere in engineering. Your thermostat, your fridge and your robot all use two thresholds to avoid chattering.

## States: remembering what you were doing

A robot often needs to remember what it's doing. Backing away from a wall takes time. A **state machine** keeps one named state at a time, and decides when to change it. This one drives forward, then backs off, then turns, then drives again.

```python
STOP_DISTANCE_CM = 10

state = "forward"
timer = 0                                  # loops spent in this state

def step(distance):
    global state, timer
    timer += 1
    if state == "forward":
        if distance is not None and distance < STOP_DISTANCE_CM:
            state, timer = "backoff", 0
    elif state == "backoff":
        if timer >= 3:                     # reverse for 3 loops
            state, timer = "turn", 0
    elif state == "turn":
        if timer >= 4:                     # turn for 4 loops
            state, timer = "forward", 0
    return state

# A made-up run: clear, clear, a wall appears, then it is gone again
readings = [None, 50, 30, 8, 8, 8, 8, 8, 40, 60, 60]
for d in readings:
    print("{:>4}".format(str(d)), "->", step(d))
```

Each state does one job, and the rules for leaving each state are in one place. When a robot misbehaves, you look at what state it was in and why it left. That's far easier than untangling a wall of `if` statements.

## Try it Yourself

1. Change `CLEAR_DISTANCE_CM` to 12. Does the dead zone still prevent flicker in the noisy example?
2. Change the backoff time from 3 loops to 6.
3. Challenge: add a fifth state called `"celebrate"` that runs for two loops after a turn, before going forward.

## Common Issues

**Problem**: The robot stutters when it's near an obstacle.
**Solution**: Use two thresholds with a gap between them.
**Why**: One threshold flips on every tiny wobble in the readings.

**Problem**: The robot gets stuck in one state.
**Solution**: Print the state on every loop and check its exit condition can actually happen.
**Why**: A state with an exit that depends on a reading the robot can no longer see will never end.

## Check your understanding

{% quiz %}
? Why does Kev's robot use a stop distance of 10 cm and a clear distance of 20 cm?
- [ ] The sensor can only read those two values
- [x] The gap stops the robot flipping between stop and go on noisy readings
- [ ] It saves battery
> Two thresholds create a dead zone. That is called hysteresis.

? What does a state machine do?
- [ ] Makes the code run faster
- [x] Keeps one named state at a time and decides when to change it
- [ ] Stores sensor readings
> Each state does one job, so behaviour is easier to follow and debug.
{% endquiz %}
