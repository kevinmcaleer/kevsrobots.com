---
title: Functions
description: Make your own blocks in Snakie with functions and inputs, and see how they become Python def statements
layout: lesson
type: page
cover: /learn/snakie_blocks/assets/banners/07_functions.jpg
date_updated: 2026-11-20
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

You've drawn a square, a triangle and a hexagon. Each time you rebuilt nearly the same blocks. A **function** lets you build something once, give it a name, and use it as often as you like.

---

## Make your own block

Open the **Functions** drawer.

![The Functions drawer](/learn/snakie_blocks/assets/toolbox_functions.jpg){:class="img-fluid w-100"}

Drag out the first block, **to do something**. Click the name and change it to `polygon`. Whatever you put in its mouth becomes the body of your function.

Now open **Functions** again. There's a new block in the drawer called **polygon**. You just made a block.

## Give it inputs

A function gets really useful when you can hand it values. Click the **blue gear** on your function block and drag in two inputs. Name them `sides` and `size`.

The inputs become variables you can use inside the function. Build this:

![The polygon function as blocks](/learn/snakie_blocks/assets/blocks_polygon.jpg){:class="img-fluid w-100"}

On the left are three **calls** to the function. On the right is the **definition**, the recipe they all share.

{% capture snakie_code %}
import turtle

def polygon(sides, size):
    for _ in range(sides):
        turtle.forward(size)
        turtle.right(360 / sides)

polygon(3, 100)
polygon(4, 100)
polygon(6, 100)
{% endcapture %}
{% include snakie.html code=snakie_code name="polygon.py" view="split" %}

![A triangle, square and hexagon drawn by one function](/learn/snakie_blocks/assets/polygon_run.jpg){:class="img-fluid w-100"}

## Reading the Python

- `def polygon(sides, size):` *defines* the function. `def` is short for define. Nothing is drawn yet
- The indented lines are the body
- `polygon(3, 100)` *calls* it, with `sides` set to 3 and `size` set to 100
- `360 / sides` is the sum from the turtle lesson, now done by the program

Three calls, three shapes, one recipe. If you want every shape drawn with a thicker pen, you change one place.

## Functions that give something back

Some functions work out an answer. The second block in the drawer has a **return** socket for that. Here's one a robot might use:

{% capture snakie_code %}
def percent_to_duty(percent):
    return int(percent * 65535 / 100)

print(percent_to_duty(50))
print(percent_to_duty(100))
{% endcapture %}
{% include snakie.html code=snakie_code name="duty.py" view="split" %}

You met that sum in the last lesson. Wrapped in a function, it has a name that says what it's for.

## Why bother?

| Without functions | With functions |
|---|---|
| The same blocks copied again and again | One definition, many calls |
| Fix a bug in every copy | Fix it once |
| A long stack that's hard to read | Short stacks with names that explain themselves |
{:class="table table-single"}

---

## Try it Yourself

1. Add a call that draws an octagon.
2. Add a third input, `colour`, and set the pen colour inside the function.
3. Write a `beep(times)` function that plays a short tone that many times.
4. Challenge: put a call to `polygon` inside a repeat block, turning 10 degrees after each one. What do you get?

## Common Issues

**Problem**: Nothing happens when I run it.
**Solution**: Add a call block. A definition on its own does nothing.
**Why**: `def` only teaches Python the recipe. Calling it is what cooks the meal.

**Problem**: `TypeError: function takes 2 positional arguments but 1 were given`
**Solution**: Fill in every socket on the call block.
**Why**: The function needs a value for each of its inputs.

**Problem**: I renamed the function and the calls broke.
**Solution**: Rename it on the definition block. The call blocks follow.
**Why**: The calls are tied to the definition, so one rename updates them all.

## Check your understanding

{% quiz %}
? What does `def polygon(sides, size):` do?
- [ ] Draws a polygon straight away
- [x] Defines a function to use later
- [ ] Imports the turtle
> `def` stores the recipe. Nothing runs until the function is called.

? In `polygon(6, 100)`, what is `sides`?
- [x] 6
- [ ] 100
- [ ] 360
> Values are matched to inputs in order, so `sides` is 6 and `size` is 100.

? What does `return` do?
- [ ] Ends the whole program
- [ ] Goes back to the top of a loop
- [x] Hands a value back to whoever called the function
> A returning function is a value block: you can drop its call into a socket.
{% endquiz %}
