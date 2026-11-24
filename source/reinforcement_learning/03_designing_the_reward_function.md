---
title: Designing the Reward Function
description: Craft rewards that teach the right behaviour, and discover the sneaky ways a poorly designed reward can backfire.
layout: lesson
code_target: cpython
type: page
cover: /learn/reinforcement_learning/assets/banners/03_designing_the_reward_function.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to write a reward function for a robot task, predict which loopholes it leaves open, and check it by scoring different behaviours before you ever train.

---

The reward function is the most powerful lever you have in reinforcement learning, and the most dangerous. It is the whole channel through which you tell the agent what you want. Get it right and the robot learns what you meant. Get it subtly wrong and you get a robot that scores top marks by doing something completely useless.

---

## What makes a good reward?

A reward should answer one question: **was that a step toward the goal?** Good rewards are:

- **Immediate**: feedback arrives close in time to the action that earned it
- **Proportional**: bigger outcomes get bigger numbers
- **Specific**: tied to facts the environment can observe, not to vague intentions
- **Balanced**: dense enough to give a signal, not so noisy that it confuses

For our obstacle-avoiding BurgerBot we want three things: keep moving, do not hit anything, and reach the goal.

---

## Test rewards before you train

Here is a habit worth building: before training anything, write down a few behaviours and score them with your reward function. If a useless behaviour beats a useful one, the reward is broken.

This program scores four hand-written behaviours (30 steps each) under two reward functions: one that only punishes crashes, and a balanced one.

```python
def collision_only(event):
    return -10.0 if event == "crash" else 0.0


def balanced(event):
    if event == "crash":
        return -10.0
    if event == "goal":
        return 50.0
    if event == "forward":
        return 1.0
    return -0.1          # turning or idling costs a little


BEHAVIOURS = {
    "statue (never moves)": ["idle"] * 30,
    "spinner (turns forever)": ["turn"] * 30,
    "driver (reaches the goal)": ["forward"] * 8 + ["goal"],
    "bumper (hits a wall)": ["forward"] * 3 + ["crash"],
}

print(f"{'behaviour':<28}{'collision-only':>16}{'balanced':>10}")
for name, events in BEHAVIOURS.items():
    naive = sum(collision_only(e) for e in events)
    good = sum(balanced(e) for e in events)
    print(f"{name:<28}{naive:>16.1f}{good:>10.1f}")
```

The output:

```
behaviour                     collision-only  balanced
statue (never moves)                     0.0      -3.0
spinner (turns forever)                  0.0      -3.0
driver (reaches the goal)                0.0      58.0
bumper (hits a wall)                   -10.0      -7.0
```

Look at the first column. Under the collision-only reward the statue, the spinner and the driver all score exactly 0, so the agent has **no reason to drive**. In the second column the driver wins by a mile and both useless behaviours lose points.

---

## Reward hacking: when robots find loopholes

Agents are ruthlessly literal. They optimise what you *measure*, not what you *meant*.

**The statue problem.** Only punish crashes and the safest strategy is never to move. The robot is not stupid; it found the best policy for the reward you wrote.

**The spinner problem.** Reward "progress" without any cost for turning and a symmetric arena may let the robot collect small rewards for wobbling back and forth. Again the reward, not the robot, is at fault.

The fix is a reward with several signals at carefully chosen scales:

| Event | Reward | Why |
|---|---|---|
| Crash | -10 | Avoiding damage comes first |
| Reach the goal | +50 | Finishing is worth some risk |
| Move forward | +1 | Encourages progress |
| Turn, stop or idle | -0.1 | Small cost so idling never wins |
{:class="table table-single"}

The relative scale matters as much as the signs. A crash penalty of -10 against a goal bonus of +50 says that reaching the goal is worth some risk, but not unlimited risk.

---

## Reward shaping

Sometimes the goal is so far away that a random agent never reaches it, so it never sees the big reward and cannot learn. This is the **sparse reward problem**. The cure is **reward shaping**: add small intermediate rewards that point the way.

```python
distances = [6, 5, 4, 3, 2, 1, 0]      # distance to the goal after each forward move


def sparse_reward(distance):
    return 50.0 if distance == 0 else 0.0


def shaped_reward(distance, previous_distance):
    if distance == 0:
        return 50.0
    progress = previous_distance - distance      # positive when we got closer
    return progress * 2.0 - 0.05                 # reward progress, tiny step cost


print("move  distance  sparse  shaped")
for move in range(1, len(distances)):
    d, prev = distances[move], distances[move - 1]
    print(f"{move:>4}  {d:>8}  {sparse_reward(d):>6.1f}  {shaped_reward(d, prev):>6.2f}")
```

```
move  distance  sparse  shaped
   1         5     0.0    1.95
   2         4     0.0    1.95
   3         3     0.0    1.95
   4         2     0.0    1.95
   5         1     0.0    1.95
   6         0    50.0   50.00
```

The sparse reward says nothing until the very last move. The shaped reward says "yes, closer" at every step. But shaping can open new loopholes, for example a robot that oscillates to farm "progress", so always re-test after adding it.

> **Note:** our simulated robot knows its distance to the goal because the simulator tells it. A real robot would need a way to measure that, which is another reason this course trains in simulation.

---

## Try it Yourself

1. Add a fifth behaviour called `"dither"` to `BEHAVIOURS`: alternate `"forward"` and `"turn"` for 30 steps. How does it score under `balanced`? Is that what you wanted?
2. Change the turning cost from `-0.1` to `0.0` and rerun. Which useless behaviour now ties with the best one?
3. Write a reward for a robot arm that must pick up a block and drop it in a bin. List the events and give each a number, then invent two behaviours and score them.
4. Challenge: in the shaping example, change `progress * 2.0` to `abs(progress) * 2.0`. What loophole does that open?

## Common Issues

- **Problem**: My agent learned to stay still and never moves.
- **Solution**: Add a small negative reward for idling and a positive reward for moving forward.
- **Why**: With no cost for doing nothing, standing still is a perfectly good strategy.

- **Problem**: My agent crashes constantly and never improves.
- **Solution**: Make sure the goal reward is reachable, by placing the goal closer in early tests or adding shaping.
- **Why**: If the agent never sees a positive reward, it has nothing to move toward.

- **Problem**: It reaches the goal but takes a silly route.
- **Solution**: Add a tiny cost per step, such as `-0.05`.
- **Why**: Without a cost for time, a 200-step route and a 20-step route earn the same.

## Check your understanding

{% quiz %}
? Under the collision-only reward, why did the statue and the driver score the same?
- [ ] The driver crashed
- [x] Neither crashed, and nothing rewarded reaching the goal
- [ ] The statue moved forward
> The collision-only reward is 0 unless you crash, so useless and useful behaviours tie.

? What is reward shaping for?
- [x] Giving small intermediate rewards so the agent can find the goal
- [ ] Making the goal reward smaller
- [ ] Removing the crash penalty
> Shaping helps with sparse rewards by giving signal on the way to the goal.

? Why score hand-written behaviours before training?
- [ ] It makes training faster
- [x] It reveals loopholes where a useless behaviour beats a useful one
- [ ] The agent needs the scores to start
> If a statue outscores a driver, no amount of training will fix the reward.
{% endquiz %}

---

Next up: how to turn continuous sensor readings into the discrete states a Q-table can use.
