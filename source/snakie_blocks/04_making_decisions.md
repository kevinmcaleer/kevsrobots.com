---
title: Making Decisions
description: Use if and else blocks with comparisons in Snakie to make a dice game, and learn the Python they write
layout: lesson
type: page
cover: /learn/snakie_blocks/assets/banners/04_making_decisions.jpg
date_updated: 2026-11-20
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Robots have to decide things. Is the wall too close? Is the button pressed? In blocks and in Python, decisions are made with **if**.

---

## The if block

Open **Control**. The **if** block has a pointed socket for a question and a mouth for the blocks to run when the answer is yes.

![The Control drawer](/learn/snakie_blocks/assets/toolbox_control.jpg){:class="img-fluid w-100"}

Questions come from the **Logic** drawer. The comparison block asks things like "is this equal to that?" or "is this less than that?", and its answer is always `True` or `False`.

## Build a dice game

1. Create a variable called `roll`.
2. Drag **set roll to** onto the canvas. From **Maths**, drop **random integer from 1 to 6** into it.
3. From **Text**, add a **print** block. Type `You rolled` in it, press its **+** and drop the `roll` variable into the new socket.
4. Add an **if** block. Click its blue gear and add an **else** part.
5. From **Logic**, drop a comparison block into the if's socket and build `roll = 6`.
6. Put a **print** in each mouth: `You win!` for the if, `Try again` for the else.

![The dice program as blocks](/learn/snakie_blocks/assets/blocks_dice.jpg){:class="img-fluid w-100"}

Run it a few times. The answer is in the console.

{% capture snakie_code %}
import random
roll = random.randint(1, 6)
print("You rolled", roll)
if roll == 6:
    print("You win!")
else:
    print("Try again")
{% endcapture %}
{% include snakie.html code=snakie_code name="dice.py" view="split" %}

## Reading the Python

- `random.randint(1, 6)` picks a whole number from 1 to 6
- `if roll == 6:` is the question. Notice the **two** equals signs
- `else:` is what happens when the answer is no

### One equals or two?

This catches everyone once. The block says `=`, but the Python says `==`.

| Python | Means | Block |
|---|---|---|
| `roll = 6` | Put 6 in the box called roll | set roll to 6 |
| `roll == 6` | Is roll equal to 6? | roll = 6 (pointed) |
{:class="table table-single"}

Blocks can use one symbol for both because the *shape* tells them apart. Python needs two different spellings.

## More questions

The comparison block's dropdown has six questions:

| Block | Python | True when |
|---|---|---|
| = | `==` | Both sides are the same |
| ≠ | `!=` | They are different |
| < | `<` | The left is smaller |
| ≤ | `<=` | The left is smaller or the same |
| > | `>` | The left is bigger |
| ≥ | `>=` | The left is bigger or the same |
{:class="table table-single"}

You can join questions with the **and** / **or** block, and flip one with **not**.

---

## Try it Yourself

1. Change the game so you win on a 5 *or* a 6.
2. Use the if block's gear to add an **else if** that prints `So close!` for a 5.
3. Put the whole game inside a **repeat 10 times** block.
4. Challenge: create a `wins` variable and count how many times you win in 10 rolls.

## Common Issues

**Problem**: The comparison block won't fit in the print block.
**Solution**: Put it in the **if** block's pointed socket.
**Why**: Pointed blocks are true-or-false answers, and they belong in question-shaped sockets.

**Problem**: I typed `if roll = 6:` in the code pane and got a `SyntaxError`.
**Solution**: Use `==` to compare.
**Why**: A single `=` stores a value. Python won't let you do that inside a question.

**Problem**: It prints both messages.
**Solution**: Make sure the second print is inside the **else** mouth, not underneath the if block.
**Why**: Blocks under the if block run every time, whatever the answer was.

## Check your understanding

{% quiz %}
? Which Python asks "is roll equal to 6?"
- [ ] `roll = 6`
- [x] `roll == 6`
- [ ] `roll := 6`
> Two equals signs compare. One equals sign stores.

? When do the blocks inside "else" run?
- [ ] Always
- [x] Only when the question's answer is False
- [ ] Only when the question's answer is True
> Else is the "otherwise" branch.

? What can `random.randint(1, 6)` give you?
- [ ] Any number from 0 to 6
- [ ] Any number from 1 to 5
- [x] Any whole number from 1 to 6
> Both ends are included, just like a real dice.
{% endquiz %}
