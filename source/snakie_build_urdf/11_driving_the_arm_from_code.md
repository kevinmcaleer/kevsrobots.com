---
title: Driving the Arm from Code
description: Write MicroPython that moves servos and watch the 3-D robot in Snakie follow it on the simulated device
layout: lesson
type: page
cover: /learn/snakie_build_urdf/assets/banners/11_driving_the_arm_from_code.jpg
date_updated: 2026-11-21
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

## Your code, your robot

This is the lesson everything has been building towards. You'll write a short MicroPython program, press **Run**, and the arm in the Build workspace will move. No servos, no board, no wires.

It works because of the bindings from the last lesson. When your code moves the servo on pin 1, Snakie looks up which joint pin 1 drives and turns the shoulder to match.

---

## One servo per joint

Snakie's `instruments` module has a helper made for this: `servo_on(pin)`. It gives you a servo on that pin.

```python
import instruments as inst

shoulder = inst.servo_on(1)
shoulder.angle(140)
```

`angle()` takes **servo degrees**: 0 to 180, with 90 in the middle. From the table in the last lesson, 140 on the servo is 50 on the joint.

On a real Pico the same two lines drive a real servo wired to GP1. On the simulated device they move the model. The code doesn't change.

---

## The wave program

Switch to the **Code** workspace, make a new file called `arm.py` and type this in. You can also press Run right here to see what it prints.

{% capture snakie_code %}
import instruments as inst
import time

waist = inst.servo_on(0)
shoulder = inst.servo_on(1)
elbow = inst.servo_on(2)


def move(w, s, e):
    waist.angle(w)
    shoulder.angle(s)
    elbow.angle(e)


move(90, 90, 90)   # rest: every servo at its middle
time.sleep(1)

for wave in range(3):
    move(50, 140, 130)
    time.sleep(0.6)
    move(50, 60, 10)
    time.sleep(0.6)

move(50, 140, 130)
print("done")
{% endcapture %}
{% include snakie.html code=snakie_code name="arm.py" view="code" %}

The box on this page shows lines such as `SNK SERVO 1 140`. That's the message `angle()` sends: servo on pin 1, go to 140 degrees. The full app hides those lines from the console and uses them to move the robot instead.

---

## Run it

1. Check the device menu above the console says **Simulated device**
2. Press **Run**
3. Switch to the **Build** workspace straight away

![The program in the Code workspace](/learn/snakie_build_urdf/assets/code_run.jpg){:class="img-fluid w-100"}

The arm snaps to rest, waves three times between two poses, and finishes reaching out. In Animate mode the joint sliders move too, showing each joint's angle as the code sets it.

![The 3-D arm following the program](/learn/snakie_build_urdf/assets/code_build.jpg){:class="img-fluid w-100"}

Those two poses are your `reach`-style and `tuck` poses written as servo angles. Joint -40, 50, 40 becomes servo 50, 140, 130. Joint -40, -30, -80 becomes 50, 60, 10.

---

## Think in joint angles

Adding 90 in your head gets old quickly. Let the program do it.

{% capture snakie_code %}
POSES = {
    "rest":  (0, 0, 0),
    "reach": (-40, 60, 30),
    "tuck":  (-40, -30, -80),
}


def to_servo(joint_angle):
    # joints run -90..90, servos run 0..180
    return joint_angle + 90


for name in POSES:
    servo_angles = [to_servo(j) for j in POSES[name]]
    print(name, "->", servo_angles)
{% endcapture %}
{% include snakie.html code=snakie_code name="poses.py" view="code" %}

Run it and you get the servo angles for each of the poses you saved in lesson 7. Combine the two programs and you can write `go("reach")` in your code.

---

## Why snapping isn't great

Our program jumps from one pose to the next. A real servo would lurch, and a heavy arm might topple. Smooth motion means sending lots of small steps, each a little further than the last.

You could write that loop yourself, and it's a good exercise. Snakie can also write it for you from the motion you've already built, which is the next lesson.

---

## Try it Yourself

1. Run `arm.py` and watch the arm in the Build workspace
2. Change the sleep times to `0.2`. What happens to the wave?
3. Add a fourth pose to `POSES` and print its servo angles
4. Write a `go(name)` function that looks up a pose, converts it and moves all three servos

---

## Common Issues

**Problem**: The code runs but the arm doesn't move.
**Solution**: Check the **Servos** list in the Build panel. Each pin your code uses needs a joint.
**Why**: Snakie can only move a joint if a servo pin is bound to it.

**Problem**: The wrong joint moves.
**Solution**: Match the pin numbers in your code to the bindings: 0 waist, 1 shoulder, 2 elbow.
**Why**: The pin number is the only link between a servo in code and a joint in the model.

**Problem**: `ImportError: no module named instruments` on a real board.
**Solution**: Let Snakie install it when it offers, as covered in [Snakie Instruments](/learn/snakie_instruments/).
**Why**: The simulated device has the module built in. A real board needs a copy in its `lib` folder.

---

## Check your understanding

{% quiz %}
? How does Snakie know which joint to turn when your code moves a servo?
- [ ] It guesses from the variable name
- [x] It looks up the servo's pin in the bindings
- [ ] It turns all of them
> The pin is the link. Pin 1 is bound to the shoulder, so the shoulder turns.

? What does angle() expect?
- [x] Servo degrees, 0 to 180
- [ ] Joint degrees, -90 to 90
- [ ] A time in seconds
> Servos think in 0 to 180. With the default binding, add 90 to a joint angle.

? Why do real robots move in lots of small steps?
- [ ] To use more memory
- [ ] Because servos can't reach big angles
- [x] So the motion is smooth instead of lurching
> Small steps close together look and feel like one smooth move.
{% endquiz %}
