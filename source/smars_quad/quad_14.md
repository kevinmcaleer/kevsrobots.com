---
title: Make it walk
description: Install SMARSLab on the Raspberry Pi and bring your SMARS Quad to life
layout: lesson
---

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

## How walking works

This video explains the walking cycle in Python: which legs move, in which order, so the robot always has enough feet on the ground to stay balanced.

{% include youtubeplayer.html id="90x1u1MjSbw" %}

---

## Common issues

- **Problem**: `i2cdetect` shows an empty grid
- **Solution**: Check SDA and SCL aren't swapped, and that I2C is turned on
- **Why**: The Pi can only talk to the board if both I2C wires go to the right pins

- **Problem**: The servos twitch or the Pi resets when the legs move
- **Solution**: Power the servos from their own 5V supply on the PCA9685's `V+` terminal
- **Why**: Eight servos can draw a few amps, far more than the Pi can supply

## Check your understanding

{% quiz %}
? What I2C address does the PCA9685 normally show up on?
- [ ] 0x20
- [x] 0x40
- [ ] 0x68
> `i2cdetect` shows `40` for a PCA9685 with the default address. (0x68 is the MPU-6050!)

? Why does the PCA9685 need its own power supply for the servos?
- [x] Eight servos draw much more current than the Raspberry Pi can supply
- [ ] The PCA9685 doesn't work on 5V
- [ ] To make the robot walk faster
{% endquiz %}

## What's next

- [Building a Robot Arm with Raspberry Pi and PCA9685](/learn/pca9685/): more servo control with the same board
- [Learn how to program SMARS with Arduino](/learn/smars_code/)
