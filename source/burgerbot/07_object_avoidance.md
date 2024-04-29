---
title: Object Avoidance
description: Learn how to avoid objects with BurgerBot.
layout: lesson
type: page
---

## Avoiding an object

To prevent collisions with obstacles, use bot.distance to measure the distance between the object and your bot. If the measured distance is less than a specified threshold (e.g., 5 cm), stop the bot and move it backward to maintain a safe distance from the obstacle.

```python
bot = Burgerbot()

while true:
    if bot.distance <= 5:
        bot.stop()
        bot.backward(1) 
    else:
        bot.forward(1)
```

---

## Following an object

To maintain a consistent distance from an object, first measure the distance between the object and your position. Move forward until the object is at the desired distance, then stop. If the object is closer than the desired distance, move backward until the desired distance is achieved. This process allows you to effectively follow an object while maintaining a set distance from it.

---

## Example 

<script src="https://gist.github.com/kevinmcaleer/750ca53e653f70aee0138abaa767f9fb.js"></script>

---
