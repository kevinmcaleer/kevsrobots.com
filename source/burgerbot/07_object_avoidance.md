---
title: Object Avoidance
description: Learn how to avoid objects with BurgerBot.
layout: lesson
type: page
---

## Avoiding an object

To prevent collisions with obstacles, use `bot.distance` to measure the distance between the object and your bot. If the measured distance is less than a threshold (for example 15 cm), stop the bot, back up and turn to maintain a safe distance from the obstacle.

### Understanding the Distance Sensor

The ultrasonic rangefinder works by sending out sound waves and measuring how long they take to bounce back. The `bot.distance` property returns the distance in centimeters to the nearest object.

**The code:**

```python
from burgerbot import Burgerbot

bot = Burgerbot()

while True:  # Infinite loop - runs forever (note: capital T in True!)
    distance = bot.distance  # Get current distance in cm

    if distance <= 15:  # Object closer than 15cm
        print(f"Too close! Distance: {distance}cm")
        bot.stop()
        bot.backward(0.5)   # Back up
        bot.turn_left(0.5)  # Turn away
    else:
        print(f"Path clear. Distance: {distance}cm")
        bot.forward(0.1)    # Keep going, checking 10 times a second
```

**How it works:**

1. `while True:` creates an infinite loop that runs continuously
2. `bot.distance` gets the current distance reading from the rangefinder
3. If distance is 15cm or less, the robot stops, reverses and turns left
4. Otherwise the robot keeps moving forward
5. Using `forward(0.1)` means the robot checks the distance ten times a second

**Why short bursts?** Checking often allows the robot to respond more quickly to obstacles, making navigation smoother and safer.

---

## Common Issues

- **Problem**: `bot.distance` returns 0 or very large numbers (like 400)
- **Solution**: Check that the rangefinder Trigger and Echo pins are wired correctly to GPIO pins
- **Why**: Incorrect wiring prevents the sensor from sending or receiving ultrasonic signals

- **Problem**: Robot doesn't detect objects closer than 2cm
- **Solution**: This is normal - ultrasonic sensors have a minimum detection range of about 2cm
- **Why**: Sound waves need time and space to separate transmission from echo

- **Problem**: Robot doesn't detect soft or angled objects well
- **Solution**: Use objects with hard, flat surfaces for testing. Ultrasonic sensors work best with solid, flat surfaces facing the robot
- **Why**: Soft materials (like fabric) absorb sound waves, and angled surfaces bounce them away. Colour doesn't matter to sound

- **Problem**: Robot behaves erratically, sometimes ignoring obstacles
- **Solution**: Ensure you're using `while True:` with a capital T. Python is case-sensitive!
- **Why**: `while true:` will cause a NameError because `true` is not defined in Python (it's `True`)

- **Problem**: Robot gets stuck in corners, turning back and forth
- **Solution**: Turn for a random time with `random.uniform(0.3, 1.0)` from the `random` module
- **Why**: Turning the same amount every time can lead the robot straight back to the wall it just left

---

## Try it Yourself

Ready to experiment with object avoidance? Try these challenges:

1. **Adjust the detection threshold**: Change `if distance <= 15:` to `if distance <= 30:`. How does the robot's behavior change? What about `distance <= 5`?

2. **Smarter turning**: Turn left or right at random after reversing. Hint: `import random` and `random.choice([bot.turn_left, bot.turn_right])(0.5)`.

3. **Variable speed**: Make the robot slow down as it approaches objects. Use the distance value to calculate motor speed.

4. **Obstacle counter**: Keep track of how many objects the robot has avoided. Print this count to the console.

**Advanced Challenge**: Create a robot that maintains exactly 10cm from a wall as it drives parallel to it. You'll need to constantly adjust direction based on distance readings!

---

## Following an object

To follow an object, flip the idea around. Measure the distance, then move forward if the object is too far away, backward if it's too close, and stop when it's just right. Hold your hand in front of BurgerBot and it will follow it like a pet!

---

## Example

```python
from burgerbot import Burgerbot

bot = Burgerbot()
bot.speed = 0.4

TARGET = 20     # the distance we want to keep, in cm
MARGIN = 5      # how close is 'close enough'

while True:
    distance = bot.distance
    if distance > TARGET + MARGIN:
        bot.forward(0.1)
    elif distance < TARGET - MARGIN:
        bot.backward(0.1)
    else:
        bot.stop()
```

The `MARGIN` gives the robot a 'just right' zone. Without it, the robot would jitter back and forth around the exact target distance.

---

{% quiz %}
? In the avoidance code, why do we call `bot.forward(0.1)` rather than `bot.forward(2)`?
- [ ] Short movements use less battery
- [x] So the robot checks the distance often and reacts quickly
- [ ] `forward()` only accepts values below 1
> While `forward()` is waiting, the robot isn't checking the distance. Short waits keep it responsive.

? What is `MARGIN` for in the object-following code?
- [ ] It sets the robot's speed
- [x] It creates a 'just right' zone so the robot doesn't jitter
- [ ] It limits the maximum distance
> Without a margin the robot would constantly overshoot and correct around the exact target.
{% endquiz %}

---
