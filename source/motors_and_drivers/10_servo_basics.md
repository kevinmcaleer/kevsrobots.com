---
title: Servo basics
description: Control a hobby servo with 50 Hz pulses of 1 to 2 milliseconds and learn the duty_u16 maths on the Raspberry Pi Pico.
layout: lesson
type: page
cover: /learn/motors_and_drivers/assets/banners/10_servo_basics.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to explain the 50 Hz servo pulse and convert a pulse width in microseconds into a Pico duty_u16 value.

Scout's wheels are sorted. Now for the head. A hobby servo like the SG90 needs no motor driver, since its control circuit is built in. All it wants is power, ground and a very particular pulse on its signal wire.

## Wiring

An SG90 has three wires:

| Wire colour | Job | Connect to |
|---|---|---|
| Brown (or black) | Ground | Common ground (battery negative and Pico GND) |
| Red | Power, 4.8 to 6 V | Battery positive (or 5 V) |
| Orange (or yellow) | Signal | GP10 |
{:class="table table-single"}

Power the servo from the battery pack, not from the Pico's 3V3 pin, because a servo can pull several hundred milliamps when it moves. While experimenting on USB, one small unloaded servo can run from VBUS (5 V), but if the Pico resets whenever the servo moves, switch to the battery. Always share a ground.

## The pulse

A servo listens for a pulse repeated **50 times a second**, which is one every 20 ms. The *width* of that pulse tells it the angle:

| Pulse width | Servo angle (nominal) |
|---|---|
| 1.0 ms | One end of travel |
| 1.5 ms | Centre |
| 2.0 ms | The other end of travel |
{:class="table table-single"}

The pulse is short, and most of the 20 ms period is silence. The duty cycle is tiny: 1.5 ms out of 20 ms is only 7.5%.

From the angle you want to the signal on the wire, there are a few small steps:

```mermaid
flowchart LR
    A["angle in degrees"] --> P["pulse width in microseconds"]
    P --> D["duty_u16 at 50 Hz"]
    D --> W["PWM on GP10"]
    W --> S["servo moves"]
```

## Pulse width to duty_u16

On the Pico we set the frequency to 50 Hz, then ask for a duty as a 16-bit number. The conversion is just a ratio:

```python
def pulse_to_duty(us, freq=50):
    """Convert a pulse width in microseconds to a Pico duty_u16 value."""
    period_us = 1000000 / freq          # 20000 us at 50 Hz
    return int(us / period_us * 65535)  # fraction of the period, scaled to 16 bits


for us in (500, 1000, 1500, 2000, 2500):
    print(us, "us ->", pulse_to_duty(us))
# 500 us  -> 1638
# 1000 us -> 3276
# 1500 us -> 4915
# 2000 us -> 6553
# 2500 us -> 8191
```

Walk through 1500 us: the period is 20000 us, so the pulse is 1500 / 20000 = 0.075 of the period. Multiply by 65535 and you get 4915.

## Move the servo

Run this with the servo horn removed or free to move. It should swing centre, one end, the other end, centre:

```python
from machine import Pin, PWM
from time import sleep_ms


def pulse_to_duty(us, freq=50):
    period_us = 1000000 / freq
    return int(us / period_us * 65535)


servo = PWM(Pin(10))
servo.freq(50)

try:
    for us in (1500, 1000, 2000, 1500):
        print("pulse", us, "us")
        servo.duty_u16(pulse_to_duty(us))
        sleep_ms(1000)
finally:
    servo.duty_u16(0)    # stop sending pulses; the servo relaxes
```

## From angle to pulse

We want to think in degrees, not microseconds. Map the angle 0 to 180 onto the pulse range with simple linear interpolation:

```python
def angle_to_pulse(angle, min_us=1000, max_us=2000, max_angle=180):
    """Map an angle in degrees onto a pulse width in microseconds."""
    angle = max(0, min(max_angle, angle))
    return min_us + (max_us - min_us) * angle / max_angle


print(angle_to_pulse(0))      # 1000.0
print(angle_to_pulse(90))     # 1500.0
print(angle_to_pulse(180))    # 2000.0
print(angle_to_pulse(45))     # 1250.0
```

Here is the secret that makes servos fiddly: those 1 to 2 ms numbers are a *convention*. Many SG90s actually travel further, closer to 0.5 to 2.4 ms, and every servo differs. Next lesson we calibrate your servo's real range.

## Try it Yourself

1. Change the test loop to visit 1250 and 1750 as well. Does the horn move a quarter of the way?
2. Use `angle_to_pulse` and `pulse_to_duty` together to move the servo to 30, 90 and 150 degrees.
3. Challenge: work out the duty_u16 for a 60 Hz signal at 1.5 ms. Does the servo mind?

## Common Issues

- **Problem**: The servo jitters or buzzes at rest.
- **Solution**: Check the power supply first. If that is fine, set `duty_u16(0)` when you are not moving.
- **Why**: A weak supply makes the position loop hunt, and constant pulses make the servo fight any force on the horn.

- **Problem**: The servo goes to one end and stays there, whatever I send.
- **Solution**: Check the frequency is 50 and your pulse is between roughly 500 and 2500 us.
- **Why**: A default frequency of 1 kHz sends pulses the servo cannot read.

- **Problem**: The Pico resets when the servo moves.
- **Solution**: Power the servo from the battery and join the grounds.
- **Why**: Servo movements cause a current surge that makes the Pico's supply dip.

## Check your understanding

{% quiz %}
? How often does a hobby servo expect a pulse?
- [x] 50 times a second, one every 20 ms
- [ ] 1000 times a second
- [ ] Once a second
> The pulse width, not the repeat rate, tells it the angle.

? What duty_u16 value gives a 1500 us pulse at 50 Hz?
- [x] 4915
- [ ] 7500
- [ ] 1500
> 1500 / 20000 = 0.075 of the period, times 65535 gives 4915.

? Why power the servo from the battery rather than the Pico's 3V3 pin?
- [x] A servo can pull several hundred milliamps when it moves
- [ ] Servos need exactly 3.3 V
- [ ] The battery is quieter
> Surges from the servo can make the Pico's supply dip and reset it. Always share a ground.
{% endquiz %}
