---
title: Make it walk
description: >-
    Install SMARSLab on the Raspberry Pi and bring your SMARS Quad to life
layout: lesson
type: page
cover: /learn/smars_quad/assets/banners/quad_14.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to run SMARSLab, explain a creeping walk with real numbers, and run your own walking script.

## Watch the video

{% include youtubeplayer.html id="Kf-ziDjZRx0" %}

---

## Set up the Raspberry Pi

1. Turn on I2C: run `sudo raspi-config`, choose **Interface Options > I2C** and select **Yes**.
2. Check the Pi can see the PCA9685:

    ```bash
    sudo apt install -y i2c-tools python3-smbus
    i2cdetect -y 1
    ```

    You should see `40` in the grid. That's the PCA9685's address.

## Install and run SMARSLab

```bash
cd smarslab
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python smars_lab.py
```

Now open a web browser at `http://<your-pi-address>:5000` and use SMARSLab to move the legs. The `configure_limbs.py` script in the same folder lets you check which PCA9685 channel each limb is on, and wiggle a limb to identify it.

The virtual environment (`venv`) keeps SMARSLab's libraries separate from the Pi's system Python, because recent Raspberry Pi OS versions refuse to install packages into the system Python. Activate it with `source venv/bin/activate` each time you open a new terminal.

> **Teachers**: SMARSLab serves its page on port 5000 over your local network, so the Pi and the browser must be on the same network. School Wi-Fi often isolates devices from each other, so test this first. A phone hotspot or a small router can work if the school network does not. Anyone on the same network can open the page and move the robot, so keep it on a network you trust.

## How walking works

This video explains the walking cycle in Python: which legs move, in which order, so the robot always has enough feet on the ground to stay balanced.

{% include youtubeplayer.html id="90x1u1MjSbw" %}

## A creeping gait, worked through with numbers

A robot with four legs is steady while it has three feet on the ground, because three points make a triangle and the body's centre of balance can sit inside it. Lift two feet at once and the robot has to balance on two points, which is much harder. So the safe way to start is to lift one leg at a time. This is called a creeping gait.

Here is one cycle, using the leg names from lesson 13:

```mermaid
flowchart LR
    A["1 Lift FL, swing forward, lower"] --> B["2 Lift BR, swing forward, lower"]
    B --> C["3 Lift FR, swing forward, lower"]
    C --> D["4 Lift BL, swing forward, lower"]
    D --> E["5 All four legs sweep back together"]
    E --> A
```

The order FL, BR, FR, BL always lifts a leg that is diagonally across from the last one, which keeps the balance more even than going round in a circle. Now the arithmetic. Suppose lifting, swinging and lowering a foot takes 0.6 s (three moves of 0.2 s each) and the sweep back takes 0.4 s:

- The cycle takes 4 x 0.6 s + 0.4 s = 2.8 s
- Each foot is off the ground for 0.6 s out of 2.8 s, which is 0.6 / 2.8 = 21 per cent of the time
- At every moment at least 3 of the 4 feet are down, and during the sweep all 4 are
- If each leg swings 40 degrees (for example from 70 to 110 degrees), the body moves forward by the length of that swing once per cycle

To go faster, shorten the pauses or use a bigger swing. To make the gait more stable, slow it down so the body has time to settle before the next lift.

## Your own walking script

SMARSLab has its own walking code, which the video above covers. To see how the same idea works from scratch, save this as `quad_walk.py` in the `smarslab` folder and run it with `python3 quad_walk.py` (with the `venv` active). It uses the channel table from lesson 13. The angles are starting values, not answers: calibrate them for your own robot.

```python
import time
from Adafruit_PCA9685 import PCA9685

# Channels from lesson 13.
LEG = {"FL": 0, "BL": 2, "BR": 4, "FR": 6}
FOOT = {"FL": 1, "BL": 3, "BR": 5, "FR": 7}

# Calibrate these for your robot (lessons 11 and 13).
LEG_CENTRE = 90         # leg pointing straight out
SWING = 20              # degrees each side of centre
FOOT_DOWN = 100         # foot on the ground
LIFT = 30               # degrees to lift the foot
# +1 or -1 per limb. Flip a sign if that limb moves the wrong way.
LEG_SIGN = {"FL": 1, "BL": 1, "BR": -1, "FR": -1}
FOOT_SIGN = {"FL": 1, "BL": 1, "BR": 1, "FR": 1}

LOW, HIGH = 50, 150     # never go outside this window
STEP_PAUSE = 0.2        # seconds for each move
ORDER = ["FL", "BR", "FR", "BL"]
CYCLES = 3

pwm = PCA9685(busnum=1)
pwm.set_pwm_freq(50)


def angle_to_count(angle):
    angle = max(LOW, min(HIGH, angle))
    return round((1.0 + angle / 180) / 20.0 * 4096)


def move(channel, angle):
    pwm.set_pwm(channel, 0, angle_to_count(angle))


def leg_angle(name, forward):
    # forward is -1 (back) or +1 (forward)
    return LEG_CENTRE + LEG_SIGN[name] * SWING * forward


def foot_angle(name, up):
    return FOOT_DOWN + FOOT_SIGN[name] * LIFT * up


def stand():
    for name in ORDER:
        move(FOOT[name], foot_angle(name, 0))
        move(LEG[name], leg_angle(name, -1))
        time.sleep(0.3)


def step_leg(name):
    move(FOOT[name], foot_angle(name, 1))       # lift
    time.sleep(STEP_PAUSE)
    move(LEG[name], leg_angle(name, 1))         # swing forward
    time.sleep(STEP_PAUSE)
    move(FOOT[name], foot_angle(name, 0))       # lower
    time.sleep(STEP_PAUSE)


def sweep_back():
    for name in ORDER:
        move(LEG[name], leg_angle(name, -1))    # all four push the body forward
    time.sleep(STEP_PAUSE * 2)


try:
    stand()
    for cycle in range(CYCLES):
        print("Cycle", cycle + 1, "of", CYCLES)
        for name in ORDER:
            step_leg(name)
        sweep_back()
finally:
    for channel in range(8):
        pwm.set_pwm(channel, 0, 0)              # release the servos
    print("Done")
```

Start with the robot on blocks, feet off the table, and watch one cycle. Only then put it on the floor. If a leg swings the wrong way, flip its sign in `LEG_SIGN`. If a foot lifts into the table instead of away from it, flip its sign in `FOOT_SIGN`.

## Try it Yourself

1. Run `quad_walk.py` on blocks and watch the order the legs move in. Does it match the diagram?
2. Change `SWING` from 20 to 10 and then to 30. How does the robot's step length change? Check the foot window is respected.
3. Change `ORDER` to go round in a circle (FL, FR, BR, BL). Is the robot as steady?
4. Challenge: work out the cycle time when `STEP_PAUSE` is 0.1, and what fraction of the time each foot is lifted.

## Common Issues

- **Problem**: `i2cdetect` shows an empty grid
- **Solution**: Check SDA and SCL aren't swapped, and that I2C is turned on
- **Why**: The Pi can only talk to the board if both I2C wires go to the right pins

- **Problem**: The servos twitch or the Pi resets when the legs move
- **Solution**: Power the servos from their own 5V supply on the PCA9685's `V+` terminal
- **Why**: Eight servos can draw a few amps, far more than the Pi can supply

- **Problem**: `ModuleNotFoundError: No module named 'Adafruit_PCA9685'`
- **Solution**: Activate the virtual environment with `source venv/bin/activate` and run `pip install -r requirements.txt` again
- **Why**: The library is installed inside the `venv`, not in the system Python

- **Problem**: The robot falls over or drags its feet when it walks
- **Solution**: Slow it down with a larger `STEP_PAUSE`, lift the feet further with a bigger `LIFT`, and check the horns are centred
- **Why**: The body needs time to settle on three feet, and a foot that barely lifts scrapes the floor

## Check your understanding

{% quiz %}
? What I2C address does the PCA9685 normally show up on?
- [ ] 0x20
- [x] 0x40
- [ ] 0x68
> `i2cdetect` shows `40` for a PCA9685 with the default address.

? In the creeping gait, how many feet are on the ground while one leg steps?
- [ ] 1
- [ ] 2
- [x] 3
> Only one leg is lifted at a time, so three feet stay down.

? With 0.6 s per leg step and a 0.4 s sweep, how long is one cycle?
- [x] 2.8 seconds
- [ ] 1.0 second
- [ ] 4.0 seconds
> 4 x 0.6 + 0.4 = 2.8 seconds.
{% endquiz %}

## What's next

Well done, you have built a walking robot! You now know how a PCA9685 lets one small computer control lots of servos, and how a gait keeps a four-legged robot upright. Keep going with these:

- [Building a Robot Arm with Raspberry Pi and PCA9685](/learn/pca9685/): more servo control with the same board
- [Learn how to program SMARS with Arduino](/learn/smars_code/)
- [Build the original tracked SMARS robot](/learn/smars/)
