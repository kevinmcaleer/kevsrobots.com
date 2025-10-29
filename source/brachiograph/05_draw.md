---
title: Draw
description: Learn how to draw with the BrachioGraph
layout: lesson
type: page
cover: assets/cover.png
---

## Draw a simple box

Type the following code:

```python
bg.box(bounds=[-2, 7, 2, 11])
```

This will draw a box with the following coordinates:

- x1: -2
- x2: 7
- y1: 2
- y2: 11

The box will be drawn in the top right corner of the drawing area.

The box will be sligthy swiggly, because the BrachioGraph is not perfectly calibrated, and because thats part of the charm of a BrachioGraph drawing. The box should be around 4cm by 4cm.

---

## Test Patterns

The BrachioGraph library has a few test patterns and images we can try out. Here are a few examples:

```python
bg.test_pattern()
```

---

Note that the grid that is drawn will be wobbley and imperfect. This is because the BrachioGraph needs to be tighted up to remove some of the slack. To do this you can attach the small screws that came with the Servos to the inner and outer arms of the BrachioGraph. This will help to remove some of the slack in the arms and make the drawing more accurate.

---

## Draw our first image

Now that we have a basic understanding of how to draw with the BrachioGraph, we can start to draw our first image. We will start with a simple image of the BrachioGraph library creator.

The BrachioGraph doesn't know how to draw pictures directly; they need to be converted into a series of points that the BrachioGraph can draw. We'll learn how to do that shortly, but first lets try to plot our first image.

```python
bg.plot_file("images/demo.json")
```

---

## Enhancing Your BrachioGraph Setup with Calibration

### Creating a Custom Configuration File

Let's start by creating a file to store your custom BrachioGraph configuration. In the same directory as `brachiograph.py`, create a new file named `custom.py`.

In `custom.py`, import the `BrachioGraph` class and create an instance with the specific pulse-width values you used earlier. Here’s an example:

```python
from brachiograph import BrachioGraph

bg = BrachioGraph(
    servo_1_parked_pw=1570,
    servo_2_parked_pw=1450,
)
```

Save this file. From now on, you can initialize your BrachioGraph in the Python shell using this custom definition. Try it out:

```python
from custom import bg
```

---

### Correcting Hysteresis

Next, we'll work on improving the machine's performance. Due to the limited power of the motors and the inherent flex in the arms and plastic gears, you might notice some slack in the system. This often causes the arms to fall short of their target positions. When approached from the opposite direction, the error appears on the other side of the target. This phenomenon is known as hysteresis.

To counteract hysteresis, we can command the motors to slightly overshoot their targets. Update your BrachioGraph definition in `custom.py` to include hysteresis correction:

```python
from brachiograph import BrachioGraph

bg = BrachioGraph(
    servo_1_parked_pw=1570,
    servo_2_parked_pw=1450,
    hysteresis_correction_1=10,
    hysteresis_correction_2=10,
)
```

Restart the Python shell, import your custom-defined BrachioGraph again:

```python
from custom import bg
```

Test the improved performance by running the test pattern in both directions:

```python
bg.test_pattern(both=True)
```

You should see better results now. Experiment with hysteresis correction values between 7 and 20 to find the optimal settings. Remember to restart the Python shell and re-import the definition each time you make changes.

---

### Fine-Tuning Angle Calculations

The default BrachioGraph definition assumes a 10 µs change in pulse-width corresponds to a 1-degree change in the motor's position. However, in practice, this might not be entirely accurate, causing distortions in your drawings. 

To correct this, first ensure that when you call `bg.park()`, the inner arm is exactly at -90˚. Then, execute:

```python
bg.set_angles(angle_1=0)
```

Check if the inner arm is precisely at 0˚. If not, adjust `angle_1` until it is. Determine the actual pulse-width value needed to reach 0˚ and subtract it from the -90˚ pulse-width value. This gives you the pulse-width difference required for a 90˚ movement. Divide this difference by 90 to find the pulse-width change per degree.

Perform a similar calibration for the outer arm. Once you have these values, update your `custom.py` file as follows:

```python
from brachiograph import BrachioGraph

bg = BrachioGraph(
    servo_1_parked_pw=1570,
    servo_2_parked_pw=1450,
    hysteresis_correction_1=10,
    hysteresis_correction_2=10,
    servo_1_degree_ms=-9.8,
    servo_2_degree_ms=10.1,
)
```

Notice that the value for `servo_1_degree_ms` might be negative. This is because one of the servos could be mounted upside-down. If the sign is incorrect, the arm will move in the opposite direction.

With these adjustments, your BrachioGraph should now function more accurately, providing more precise and consistent drawings.

---
