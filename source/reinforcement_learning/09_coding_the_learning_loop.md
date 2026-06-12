---
title: Coding the Learning Loop
description: Implement the Q-learning update rule and the full episode loop — the algorithm that turns experience into a policy.
layout: lesson
type: page
cover: assets/cover.jpg
date_updated: 2026-06-13
---

![Cover](assets/cover.jpg){:class="cover"}

---

We now have all the pieces:

- A Q-table (lesson 7) to store what we've learned
- ε-greedy action selection (lesson 8) to balance exploration and exploitation
- A discount factor γ to value future rewards (lesson 6)
- A reward function to signal success and failure (lesson 3)

This lesson ties them all together into **the Q-learning update rule** — the single equation that makes the whole thing work.

---

## The Q-Learning Update Rule

Here's the update rule written as Python with every term explained:

```python
def update_q(q_table, state, action, reward, next_state, alpha, gamma):
    """
    Apply the Q-learning update rule to a single experience tuple.

    Q-learning update (Watkins, 1989):

        Q[s][a] += alpha * (reward + gamma * max(Q[s']) - Q[s][a])

    Breaking it down term by term:

    Q[s][a]
        Our current estimate: "how good is action a in state s?"

    reward
        What we actually got for taking action a in state s.

    max(Q[s'])
        Our best estimate of how good the NEXT state is.
        We pick the MAXIMUM over all actions — this is the "greedy"
        part of Q-learning. We assume from next_state onward we'll
        always pick the best known action.

    reward + gamma * max(Q[s'])
        The "target": what Q[s][a] SHOULD be, based on this experience.
        This is called the "TD target" (Temporal Difference target).

    reward + gamma * max(Q[s']) - Q[s][a]
        The "TD error": the gap between our current estimate and the
        new evidence. Positive = we underestimated, negative = we
        overestimated.

    alpha * (...)
        The learning rate alpha controls how big a step we take
        toward the new evidence. Small alpha = conservative, learns
        slowly. Large alpha = aggressive, can be unstable.

    Args:
        q_table (defaultdict): current Q-table
        state (tuple): the state we were in
        action (str): the action we took
        reward (float): the reward we received
        next_state (tuple): the state we ended up in
        alpha (float): learning rate, typically 0.1 to 0.5
        gamma (float): discount factor, typically 0.9 to 0.99

    Returns:
        defaultdict: the updated Q-table (modified in place, also returned)
    """

    # Current estimate
    current_q = q_table[state][action]

    # Best future value from next_state
    best_future_q = max(q_table[next_state].values())

    # TD target: what the Q-value should be, given this experience
    td_target = reward + gamma * best_future_q

    # TD error: how far off our current estimate is
    td_error = td_target - current_q

    # Update Q-value by taking a step of size alpha toward the target
    q_table[state][action] += alpha * td_error

    return q_table
```

This is the complete learning rule. Everything else in the algorithm is housekeeping around this one equation.

---

## The Episode Loop

An episode runs from a starting state until either the goal is reached or a maximum number of steps is hit. Here's the full episode loop:

```python
def run_episode(env, q_table, alpha, gamma, epsilon, max_steps=200):
    """
    Run one complete episode of training.

    The agent:
    1. Observes the initial state
    2. Chooses an action (ε-greedy)
    3. Executes the action in the environment
    4. Receives a reward and observes the next state
    5. Updates the Q-table
    6. Repeats until done or max_steps reached

    Args:
        env: the environment object (see lesson 10)
        q_table: the current Q-table (modified in place)
        alpha (float): learning rate
        gamma (float): discount factor
        epsilon (float): exploration probability

    Returns:
        float: total undiscounted reward for this episode
    """
    state = env.reset()           # start fresh each episode
    total_reward = 0.0
    done = False

    for step in range(max_steps):
        # 1. Choose action using ε-greedy
        action = choose_action(q_table, state, epsilon)

        # 2. Take the action; get next state and reward
        next_state, reward, done = env.step(action)

        # 3. Update Q-table using the Q-learning rule
        update_q(q_table, state, action, reward, next_state, alpha, gamma)

        # 4. Advance to next state
        state = next_state
        total_reward += reward

        if done:
            break   # episode ends: goal reached or collision

    return total_reward
```

---

## The Training Loop

The training loop runs many episodes and tracks progress:

```python
def train(env, num_episodes=500, alpha=0.2, gamma=0.9,
          epsilon_start=1.0, decay_rate=0.995, epsilon_min=0.01,
          max_steps=200):
    """
    Train a Q-learning agent over multiple episodes.

    Returns the trained Q-table and a list of per-episode rewards
    (useful for plotting learning progress).
    """
    from collections import defaultdict

    ACTIONS = ["forward", "turn_left", "turn_right", "stop"]
    q_table = defaultdict(lambda: {a: 0.0 for a in ACTIONS})
    epsilon = epsilon_start
    rewards_history = []

    for episode in range(num_episodes):
        episode_reward = run_episode(
            env, q_table, alpha, gamma, epsilon, max_steps
        )
        rewards_history.append(episode_reward)

        # Decay epsilon after each episode
        epsilon = max(epsilon_min, epsilon * decay_rate)

        # Print progress every 50 episodes
        if (episode + 1) % 50 == 0:
            recent_avg = sum(rewards_history[-50:]) / 50
            print(
                f"Episode {episode+1:4d} | "
                f"avg reward (last 50): {recent_avg:7.2f} | "
                f"epsilon: {epsilon:.4f}"
            )

    return q_table, rewards_history
```

The `rewards_history` list lets you see the agent improving. Early episodes tend to have very negative rewards (lots of crashes); later episodes trend positive as the agent learns to avoid obstacles and reach the goal.

---

## A Note on Hyperparameters

Three numbers control how Q-learning behaves: **α (alpha)**, **γ (gamma)**, and **ε (epsilon)** with its decay. Here's how to think about each:

```python
# Learning rate alpha
alpha = 0.2   # a good starting value
# Too high (> 0.5): Q-values are unstable, oscillate wildly
# Too low (< 0.01): learns so slowly that training takes forever
# Effect: how much weight to give new experience vs old estimates

# Discount factor gamma
gamma = 0.9   # good for tasks with goals 5-20 steps away
# High (0.99): patient, values far-future rewards, slower convergence
# Low (0.5): impatient, only cares about immediate rewards
# Effect: how far into the future the agent plans

# Starting epsilon and decay
epsilon_start = 1.0   # always start fully explorative
decay_rate = 0.995    # for ~500 episode runs
epsilon_min = 0.01    # never go fully greedy
```

For the BurgerBot simulation in lesson 10, the default `alpha=0.2, gamma=0.9, epsilon_start=1.0, decay_rate=0.995` work well. You'll experiment with tuning in the "Try It Yourself" section of that lesson.

---

## Try It Yourself

These exercises use the code from this lesson. Paste everything into a single Python file to try them.

1. Create a minimal "mock environment" class that always returns a fixed next_state and reward, and run `run_episode()` against it 10 times. Print the Q-values for the one state the mock returns. Watch them converge toward `reward / (1 - gamma)` (the infinite-horizon value of a constant reward stream).

2. Modify `run_episode()` to also return the sequence of `(state, action, reward)` triples from the episode. Print a trace of the first full episode and read through it to understand what happened step by step.

3. Change alpha to 0.01 and 0.9 and run 200 episodes with a mock environment that gives reward +1 every step. Compare how many episodes it takes for Q-values to stabilise in each case.

---

## Common Issues

**"Q-values are exploding to very large numbers."**
Your alpha is too high. Try reducing it to 0.1. Also check that your reward values aren't in the hundreds — keep them in the range ±50.

**"After 500 episodes, the Q-table is still mostly zeros."**
Your episode steps are hitting `max_steps` before reaching the goal or crashing — the agent never receives any significant reward. Try a shorter maze or increase `max_steps`, and verify your reward function returns non-zero values.

**"The average reward gets better then suddenly gets worse."**
This is normal and is called **catastrophic forgetting** at the micro-scale. A spurt of exploration (when epsilon is still relatively high) can temporarily disrupt a partial policy. The trend should still be upward over the full training run.

---

Next up: building the complete grid-world simulation and watching the policy actually improve.

---
