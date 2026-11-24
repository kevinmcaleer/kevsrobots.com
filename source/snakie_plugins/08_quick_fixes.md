---
title: Quick Fixes
description: Attach a one-click quick fix to a Snakie linter warning so the lightbulb can repair the code for you
layout: lesson
type: page
cover: /learn/snakie_plugins/assets/banners/08_quick_fixes.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to attach a one-click quick fix to a linter warning so the lightbulb can repair the code for you.

## Don't just complain

ClubKit can now spot `sleep(500)`. But it knows exactly what the fix is, so why make someone type it?

A **quick fix** is a repair attached to a diagnostic. The user sees a lightbulb, clicks it, and the code is corrected.

---

## The fix helper

```python
fix(title, new_text)
```

| Part | Meaning |
|---|---|
| `title` | What the menu item says |
| `new_text` | The text to put in |
{:class="table table-single"}

Used like that, the fix **replaces the squiggled text**. That's the neatest way to work: underline exactly the code that's wrong, and the fix swaps exactly that.

You can also aim a fix somewhere else by adding `line`, `column`, `end_line` and `end_column`, all counting from 1. We'll use that in the challenge at the end.

---

## Add the fix

Add `fix` to the import:

```python
from snakie import Context, diagnostic, edit, fix, message, plugin, status
```

Then give the diagnostic a `fixes` list. The new part is the last line:

```python
        match = LONG_SLEEP.search(line)
        if match:
            found.append(diagnostic(
                number,
                f"sleep({match.group(1)}) waits {match.group(1)} seconds. Did you mean milliseconds?",
                severity="warning",
                column=match.start() + 1,
                end_column=match.end() + 1,
                source="clubkit",
                fixes=[fix(f"Change to sleep_ms({match.group(1)})", f"sleep_ms({match.group(1)})")],
            ))
```

Our squiggle covers `sleep(500)`. The fix replaces it with `sleep_ms(500)`. Because the pattern starts at the word `sleep`, the `time.` in front is left alone, and `time.sleep(500)` becomes `time.sleep_ms(500)`.

Save, Reload, and nudge `patrol.py` with a small edit so it's checked again.

---

## Use it

1. Click on the squiggled `sleep(500)`
2. Press **Ctrl + .** (**Cmd + .** on a Mac), or click the lightbulb that appears at the start of the line

![The quick fix menu](/learn/snakie_plugins/assets/quickfix.jpg){:class="img-fluid w-100"}

A small **Quick Fix** menu opens with your title in it.

3. Press **Enter**

![The line after the fix](/learn/snakie_plugins/assets/fixed.jpg){:class="img-fluid w-100"}

The line now reads `time.sleep_ms(500)` and the squiggle has gone. Nobody had to remember the name of the function.

The fix is a normal edit, so **Ctrl + Z** undoes it.

---

## What makes a good fix

**The title says what will happen.** "Change to sleep_ms(500)" is better than "Fix".

**It's certain.** Only offer a fix when you're confident it's what the user wants. For a guess, leave the fix off and let the message do the work.

**It's small.** Replace the few characters that are wrong. Fixes have to name a range, so you can't replace the whole file, and that's a good thing.

**It doesn't create a new problem.** Our fix is safe because `time.sleep_ms` exists whenever `time.sleep` does. What about code that says `from time import sleep` and then calls `sleep(500)`? The fix would write `sleep_ms(500)`, and `sleep_ms` was never imported. That's the challenge below.

---

## More than one fix

`fixes` is a list, so you can offer a choice:

```python
fixes=[
    fix("Change to sleep_ms(500)", "sleep_ms(500)"),
    fix("Change to sleep(0.5)", "sleep(0.5)"),
]
```

Both appear in the menu. Keep it to two or three. A long menu is a sign the rule isn't sure what it wants.

---

## Try it Yourself

1. Add the second fix above, working out the seconds from the number that was matched. `int(match.group(1)) / 1000` gives the value
2. Undo a fix with Ctrl + Z and apply the other one
3. Challenge: when the file contains `from time import sleep`, change the title to warn that `sleep_ms` needs importing too. `"from time import sleep" in ctx.file.content` is all the check you need

---

## Common Issues

**Problem**: The lightbulb says there are no quick fixes.
**Solution**: Check `fixes=[...]` is inside the `diagnostic(...)` brackets, then Reload and edit the file.
**Why**: A fix belongs to a diagnostic. A diagnostic without `fixes` is only a message.

**Problem**: The fix replaces the wrong text.
**Solution**: Make the squiggle cover exactly the text you want replaced, by checking `column` and `end_column`.
**Why**: A fix with no range of its own replaces whatever the diagnostic underlined.

**Problem**: The fix leaves a stray bracket behind.
**Solution**: Make sure your pattern includes the closing bracket.
**Why**: `match.end()` is where the pattern stopped. Anything after it is left in place.

---

## Check your understanding

{% quiz %}
? A fix has no line or column of its own. What does it replace?
- [ ] The whole file
- [ ] The whole line
- [x] The text the diagnostic underlined
> With no range given, a fix replaces the diagnostic's own range.

? Where do you attach a fix?
- [x] In the fixes list of a diagnostic
- [ ] In a separate command
- [ ] In the status bar
> Fixes belong to diagnostics and appear on the lightbulb for that problem.

? When should you leave the fix off a warning?
- [ ] When the message is long
- [x] When you can't be sure what the user intended
- [ ] When the severity is warning
> Only offer a fix you're confident in. Otherwise explain the problem and let the user decide.
{% endquiz %}
