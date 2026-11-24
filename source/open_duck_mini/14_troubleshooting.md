---
title: Troubleshooting
description: The common Open Duck Mini problems in one place, from missing servos and a silent IMU to an oscillating gait, a leaning duck and laggy servos, with the fix and the reason for each
layout: lesson
type: page
cover: /learn/open_duck_mini/assets/banners/14_troubleshooting.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Every walking robot misbehaves sometimes. That's not a sign you did something wrong. It's just what happens when 14 servos, a sensor, a computer and a neural network all have to agree. This lesson collects the common problems in one place.

---

## Work from the bottom up

When something goes wrong, resist the urge to change everything at once. Work through the layers in order, the same way we built them:

1. **Power and wiring.** Is everything plugged in and powered?
2. **Each part on its own.** Do the tests from [lesson 9](/learn/open_duck_mini/09_testing_imu_motors_and_controller.html) still pass?
3. **Calibration.** Are the IMU calibration, `imu_upside_down` and the offsets from [lesson 10](/learn/open_duck_mini/10_calibration_and_offsets.html) right?
4. **The policy.** Only then suspect the walking brain.

Most problems turn out to be in the first three layers.

---

## The problem table

| Problem | Solution | Why |
|---|---|---|
| A servo is not found, or the wrong joint moves | Run `check_motors.py`. Reconfigure the servo on its own with `python configure_motor.py --id <id>`, using the ID table in lesson 4 | Each joint is addressed by its ID. A servo with the wrong or a duplicate ID won't answer as the runtime expects |
| Several servos in a row are missing | Check the daisy-chain cable just before the first missing one | A loose cable cuts off every servo after it on the chain |
| The IMU is missing from `i2cdetect -y 1` | Check I2C is on in `raspi-config`, your user is in the `i2c` group, and SDA is on pin 3, SCL on pin 5, VIN on pin 1 | The BNO055 talks I2C. It usually shows at 0x28 once I2C is on and it's wired correctly |
| The gait oscillates badly, wobbling more and more | Check `imu_upside_down` in `~/duck_config.json` matches how the IMU is mounted | If the runtime thinks the IMU is the other way up, the policy corrects tilts the wrong way |
| The duck leans or drifts one way | Hang it up and run `find_soft_offsets.py` again. Check the good-pose list from lesson 10 | Small offset errors in hips, knees or ankles add up to a lean |
| The foot switches are out of sync | Lower the duck onto a flat table and adjust the offsets until both switches click at the same time | The policy uses the foot switches to know which foot is down |
| The Xbox controller won't pair or connect | In `bluetoothctl`, run `scan on`, then `pair`, `trust` and `connect` with its MAC. Test with `xbox_controller.py` | Only paired and trusted devices reconnect by themselves |
| The servos feel laggy and the walk is jerky | Check the udev rule `/etc/udev/rules.d/99-usb-serial.rules` is in place, then reboot. Check your servo board uses an FTDI chip | The rule sets the FTDI latency timer to 1. It only matches `ftdi_sio`, so it does nothing for other adapters |
| GPIO errors on a Raspberry Pi 5 | Inside the virtual environment, run `pip uninstall -y RPi.GPIO` then `pip install lgpio` | The Pi 5 needs `lgpio` in place of `RPi.GPIO` |
| The head moves wildly or something cracks in the head | Leave Y (head control) off | Head control is very experimental and can break the head |
| `pip install -e .` or a script fails to import | Run `workon open-duck-mini-runtime` and check you're on the `v2` branch | The runtime is installed in the virtual environment, and this course uses the `v2` code |
{:class="table table-single"}

---

## Checking the latency rule

The lag problem is easy to miss, because everything *works*, just badly. To check the rule file exists and says what it should:

```bash
cat /etc/udev/rules.d/99-usb-serial.rules
```

You should see:

```text
SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
```

If your board uses a different USB-serial chip, this rule won't match it. Ask the Open Duck community what works for your board.

---

## When to ask for help

If you've worked through the layers and you're still stuck, the Open Duck community is friendly and has seen most problems before. Check the issues on the [Open_Duck_Mini](https://github.com/apirrone/Open_Duck_Mini) and [Open_Duck_Mini_Runtime](https://github.com/apirrone/Open_Duck_Mini_Runtime) repos first, because someone may already have solved yours. When you ask, include:

- Which step fails, and the exact error message
- The output of the lesson 9 tests
- A short video of the duck, if it moves but moves wrongly

---

## Try it Yourself

1. Make a copy of `~/duck_config.json` once your duck walks well, so you can always get back to a known good setup.
2. Compare your `~/duck_config.json` with `example_config.json` and note every setting you changed, and why.
3. Keep a build log of every fix you make. Future you will thank you.
