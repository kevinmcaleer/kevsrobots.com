---
title: Rules for Your Hardware
description: Teach your Snakie linter about the Raspberry Pi Pico's reserved pins, ignore comments, and decide what deserves an error
layout: lesson
type: page
cover: /learn/snakie_plugins/assets/banners/09_rules_for_your_hardware.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to teach your linter about the Pico's reserved pins, ignore comments, and decide what deserves an error.

## Rules a general linter can't know

`ruff`, the Python linter built into Snakie, knows Python. It has no idea what your club's robot is wired to, or which pins are off limits on your board. **You** do. Rules about your own hardware are where a plugin earns its keep.

Here's one for the Raspberry Pi Pico. A few GPIO numbers exist in MicroPython but aren't on the pin header, because the board uses them itself:

| Pin | What the Pico uses it for |
|---|---|
| GP23 | Controlling the power supply mode |
| GP24 | Sensing USB power |
| GP25 | The on-board LED (fine to use, on a plain Pico) |
| GP29 | Reading the supply voltage |
{:class="table table-single"}

Look at `patrol.py` again:

```python
sense = Pin(29, Pin.IN)
```
{: .no-snakie }

Somebody has "wired a sensor to GP29". There's no GP29 pad to wire it to. The program runs without a single error message and the sensor never works. That's the second mistake, and it's a perfect job for a linter.

---

## Add the rule

Under your other constants, add a table of the pins and why they're taken:

```python
OFF_HEADER = {23: "the power supply mode", 24: "USB power sensing", 29: "the VSYS voltage reading"}
```

Then add a second check inside the `for` loop of `club_rules`, after the sleep check:

```python
        for match in PIN_USE.finditer(line):
            pin = int(match.group(1))
            if pin in OFF_HEADER:
                found.append(diagnostic(
                    number,
                    f"GP{pin} is used by the Pico for {OFF_HEADER[pin]}. Pick another pin.",
                    severity="error",
                    column=match.start(1) + 1,
                    end_column=match.end(1) + 1,
                    source="clubkit",
                ))
```
{: .no-snakie }

Three things are new:

- **`finditer`** finds every match in the line, because one line can set up more than one pin
- **`match.start(1)`** is where the *first group* starts. That's the number in the brackets of our pattern, so the squiggle sits under `29` alone and not under all of `Pin(29`
- **`severity="error"`**. This can't work as written, so it earns the stronger red underline

There's no fix this time. ClubKit can't know which pin the sensor is really on, so it explains and lets the user choose. That's the "be certain" rule from the last lesson.

Save, Reload and nudge `patrol.py`.

![The error on GP29](/learn/snakie_plugins/assets/hover_pin.jpg){:class="img-fluid w-100"}

The `29` is underlined in red, and the hover says why. The pop-up also says there are no quick fixes available, which is exactly what we intended.

---

## Don't lint the comments

Try adding this line to `patrol.py`:

```python
# time.sleep(500) was far too long
```

It gets a squiggle. The linter is reading text, and a comment is text. That's a **false alarm**, and false alarms teach people to ignore your plugin.

The simplest cure is to cut each line off at the `#` before checking it. Add one line at the top of the loop and use `code` in place of `line` in both checks:

<!-- code-check: skip -->
```python
    for number, line in enumerate(ctx.file.content.splitlines(), start=1):
        code = line.split("#")[0]

        match = LONG_SLEEP.search(code)
        ...
        for match in PIN_USE.finditer(code):
```
{: .no-snakie }

The columns still line up, because we only chopped off the end of the line.

It isn't perfect. A `#` inside a string, such as `print("#1")`, would cut the line short. For a club linter that's a fair trade: we might miss something on a rare line, but we won't raise a false alarm. When you have to choose, **miss a problem before you invent one**.

---

## Designing a good rule

Before adding a rule, ask four questions.

1. **Is it nearly always a mistake?** If people often do it on purpose, it's a hint at most
2. **Can the message teach?** "GP29 is used by the Pico for the VSYS voltage reading. Pick another pin." says what's wrong, why, and what to do
3. **Does the squiggle point at the exact spot?** Underline the `29`, not the whole line
4. **Is it about *your* setup?** Leave general Python style to `ruff`

---

## Try it Yourself

1. Add a line using `Pin(23)` and check the message names the right reason
2. Put two pins on one line, such as `a, b = Pin(24), Pin(2)`. Is only the bad one underlined?
3. The club robot's motors are on GP2 to GP5. Add a `"warning"` when a program sets up one of those pins itself, with a message saying to use `clubbot` instead
4. Add `"hint"` for any `Pin(25` line: "GP25 is the on-board LED on a Pico. On a Pico W use Pin('LED')."

---

## Common Issues

**Problem**: The whole of `Pin(29` is underlined.
**Solution**: Use `match.start(1)` and `match.end(1)`.
**Why**: With no number, `start()` and `end()` cover the whole match. The `1` picks the first bracketed group.

**Problem**: `KeyError` or no message for a pin.
**Solution**: Check the pin is in `OFF_HEADER` with `if pin in OFF_HEADER` before looking it up.
**Why**: Looking up a key that isn't in a dictionary raises an error, and a linter that raises is skipped.

**Problem**: A warning appears on a commented-out line.
**Solution**: Check `code`, the part before the `#`, in every rule.
**Why**: It's easy to update one check and forget the other.

---

## Check your understanding

{% quiz %}
? Why is a plugin a good place for a rule about GP29?
- [ ] Plugins run faster than ruff
- [x] It's knowledge about your hardware that a general Python linter doesn't have
- [ ] Only plugins can show errors
> General linters know the language. Your plugin knows your board and your robot.

? Why does the GP29 rule have no quick fix?
- [ ] Errors can't have fixes
- [x] The plugin can't know which pin the user really meant
- [ ] Fixes only work on sleep
> A fix should be certain. Here only the user knows the right pin.

? What is a false alarm?
- [x] A warning about code that's fine
- [ ] A warning the user ignores
- [ ] A linter that crashes
> False alarms, such as warnings on comments, teach people to stop trusting the linter.
{% endquiz %}
