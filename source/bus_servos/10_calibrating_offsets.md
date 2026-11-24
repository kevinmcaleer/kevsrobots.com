---
title: Calibrating offsets
description: Teach Buddy where zero is, understand the offset and sign maths, and set safe joint limits in config.json
layout: lesson
type: page
cover: /learn/bus_servos/assets/banners/10_calibrating_offsets.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

A servo knows its own position between 0 and 360 degrees, but not what that means for your arm. Is 217 degrees "straight up" or "folded flat"? That depends on how you bolted the horn on. **Calibration** tells the code where "home" is, so that 0 degrees means the same thing on every joint.

---

## Two frames of reference

There are two sets of numbers:

- **Raw** degrees: what the servo reports. The zero is wherever the encoder happens to start
- **User** degrees: what *you* ask for. After calibration, 0 is the home pose, with the arm stretched straight up

Each joint has two numbers in its config that turn one into the other:

- `offset_deg` is the raw angle of the home pose
- `sign` is `1` or `-1`, and flips the direction. A servo mounted the other way round turns "backwards", and `-1` puts that right

Here are the two conversions, taken from `buddy.py`, as a small program you can run on the Pico or your computer with no hardware attached.

```python
def user_to_raw(offset_deg, sign, user_deg):
    # raw = offset + sign * user
    return offset_deg + sign * user_deg


def raw_to_user(offset_deg, sign, raw_deg):
    # user = (raw - offset) / sign
    return (raw_deg - offset_deg) / sign


# A joint whose home pose reads 217 degrees raw, mounted the normal way round.
offset, sign = 217.0, 1
print("Home (user 0) is raw", user_to_raw(offset, sign, 0))
print("User +30 is raw", user_to_raw(offset, sign, 30))
print("Raw 187 is user", raw_to_user(offset, sign, 187))

# The same joint mounted the other way round: sign is -1.
sign = -1
print("Flipped: user +30 is raw", user_to_raw(offset, sign, 30))
print("Flipped: raw 187 is user", raw_to_user(offset, sign, 187))
```

Run it, and you see that with `sign = 1`, user +30 is raw 247, and with `sign = -1` it's raw 187. The same command now moves the joint the opposite way. That's what `sign` is for.

## Teaching the arm where home is

The repo's `calibrate.py` does the job. It lets the arm go limp, waits for you to pose it, reads every joint, and saves the result to `config.json` on the Pico's flash. Copy `sts3215.py`, `buddy.py` and `config.py` to the Pico first.

```python
from sts3215 import STS3215   # sts3215.py, buddy.py and config.py from the repo
from buddy import Buddy
from config import default_config, load_config, save_config, CONFIG_PATH

bus = STS3215(uart_id=0, tx_pin=0, rx_pin=1, baudrate=1_000_000)
buddy = Buddy(bus, config_path=CONFIG_PATH)

buddy.set_torque_all(False)
print("Pose the arm to its zero position, then press ENTER.")
input()

captured = buddy.read_all_positions()
print("Captured:", captured)

cfg = load_config(CONFIG_PATH)
if "joints" not in cfg:
    cfg = default_config()

# new_offset = old_offset + sign * captured_user_deg
# (so the captured pose decodes to 0 next time)
for name, user_deg in captured.items():
    if user_deg is None:
        print("calibrate: skipping", name, "(no read)")
        continue
    j = cfg["joints"][name]
    j["offset_deg"] = j["offset_deg"] + j["sign"] * user_deg

save_config(cfg, CONFIG_PATH)
print("Wrote", CONFIG_PATH)
```

Here is how to use it:

1. Pose the arm in its **home pose**: all joints at 0, with the arm extended straight up. Hold it there
2. Run the program and press ENTER
3. Check the printed `Captured` numbers and look for a `None`. That means a joint didn't answer, and its offset wasn't changed

The maths in the loop is the same as above. The captured user angle is `(raw - offset) / sign`. Add `sign` times that to the old offset, and the offset becomes exactly the raw angle of your pose. Next time, that pose decodes to 0.

The file it writes looks like this, with one entry per joint:

```json
{
  "joints": {
    "base": {"id": 1, "min_deg": 0, "max_deg": 360, "offset_deg": 217.4, "sign": 1},
    "shoulder": {"id": 2, "min_deg": 0, "max_deg": 360, "offset_deg": 181.0, "sign": 1}
  },
  "wifi": {"ssid": "", "password": ""}
}
```

## A trap: the default limits

Look at `min_deg` and `max_deg`. They default to 0 and 360 in the **user** frame. Once 0 is the home pose, you need joints to go *negative* too, such as a shoulder leaning back. Buddy clamps every request to `[min_deg, max_deg]` and prints `buddy: clamped` when it does. With a minimum of 0, a request for -30 silently becomes 0.

So after calibrating, set sensible limits. There's a second rule: `offset + sign * limit` has to stay between 0 and 360 raw degrees, because that's all the servo can address. An offset of 30 with a limit of -90 would need raw -60, which doesn't exist.

This complete program sets cautious starting limits, then checks every one against that rule. It edits `config.json` directly, so run `calibrate.py` first.

```python
import json

CONFIG_PATH = "/config.json"

# (min_deg, max_deg) in user degrees. Cautious starting values: tune for your arm.
LIMITS = {
    "base":        (-90, 90),
    "shoulder":    (-90, 90),
    "elbow":       (-135, 135),
    "wrist_pitch": (-90, 90),
    "wrist_roll":  (-90, 90),
    "gripper":     (0, 45),
}

try:
    with open(CONFIG_PATH) as f:
        cfg = json.load(f)
except (OSError, ValueError):
    cfg = None

if cfg is None or "joints" not in cfg:
    print("No usable", CONFIG_PATH, "- run calibrate.py first")
else:
    problems = 0
    for name, (low, high) in LIMITS.items():
        joint = cfg["joints"].get(name)
        if joint is None:
            print("No joint called", name)
            continue
        joint["min_deg"] = low
        joint["max_deg"] = high
        # Where do the limits land in raw degrees?
        raw_a = joint["offset_deg"] + joint["sign"] * low
        raw_b = joint["offset_deg"] + joint["sign"] * high
        if min(raw_a, raw_b) < 0 or max(raw_a, raw_b) > 360:
            print("WARNING", name, "limits reach raw", raw_a, "to", raw_b,
                  "which is outside 0 to 360")
            problems += 1
        else:
            print("ok     ", name, "limits", low, "to", high,
                  "= raw", min(raw_a, raw_b), "to", max(raw_a, raw_b))
    with open(CONFIG_PATH, "w") as f:
        json.dump(cfg, f)
    print("Saved.", problems, "joint(s) need attention")
```

If a joint warns, narrow its limits, or recalibrate with the horn re-seated so the home pose is nearer the middle of the servo's turn. The gripper limits depend on which way your gripper opens, so if it closes when you expected open, flip that joint's `sign` in the config.

**Why this matters:** limits are your first line of defence against the arm driving a joint into its own body. Set them before you run any program that moves several joints.

## Try it Yourself

1. Run `calibrate.py`, then `arm_check.py` from the last lesson. At home, every joint should read close to 0.
2. Move the arm away by hand and back. Do the numbers return to near 0?
3. Challenge: change `offset_deg` for one joint by 10 in `config.json`. Rerun the check. What moved?

## Common Issues

**Problem**: A joint doesn't go to the angle I asked for, and `buddy: clamped` is printed.
**Solution**: Widen that joint's `min_deg` and `max_deg` in `config.json`.
**Why**: Requests are clamped to the limits before they're sent.

**Problem**: A joint moves the wrong way.
**Solution**: Change its `sign` from `1` to `-1`, then recalibrate.
**Why**: The servo was mounted facing the other way.

**Problem**: A joint jumps to a wild position at the edge of its range.
**Solution**: Run the limits program and fix any WARNING lines.
**Why**: If `offset + sign * limit` leaves 0 to 360, the position number sent to the servo is out of range.

---

## Check your understanding

{% quiz %}
? What does `offset_deg` store for a joint?
- [ ] The maximum angle the joint may reach
- [x] The raw servo angle of the home pose
- [ ] The servo's ID
> After calibration, the home pose decodes to user angle 0.

? What is `sign` for?
- [ ] To set how fast the joint turns
- [x] To flip a joint's direction when the servo is mounted the other way round
- [ ] To make the position positive
> `sign` is 1 or -1 and multiplies the user angle.

? Why must you widen `min_deg` after calibrating?
- [ ] The servo forgets its limits
- [x] Home is now 0, so negative angles are needed, and a minimum of 0 clamps them
- [ ] The limits are in raw degrees
> The defaults are 0 to 360, which suits an uncalibrated arm but not one with home at 0.
{% endquiz %}
