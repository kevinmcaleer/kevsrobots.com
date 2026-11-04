---
title: Viewing History
description: Explore your project's past with git log, git show and git diff
layout: lesson
type: page
navigation:
- name: Git for Beginners
- content:
  - section: Overview
    content:
    - name: Course Overview
      link: 00_intro.html
  - section: Getting Started
    content:
    - name: What is Git?
      link: 01_what-is-git.html
    - name: Installing Git
      link: 02_installing-git.html
    - name: Your First Repository
      link: 03_your-first-repository.html
  - section: Saving Your Work
    content:
    - name: Staging and Committing
      link: 04_staging-and-committing.html
    - name: Viewing History
      link: 05_viewing-history.html
    - name: Undoing Changes
      link: 06_undoing-changes.html
    - name: Ignoring Files
      link: 07_ignoring-files.html
  - section: Branches
    content:
    - name: Branches
      link: 08_branches.html
    - name: Merging and Conflicts
      link: 09_merging-and-conflicts.html
  - section: Working with GitHub
    content:
    - name: GitHub and Remotes
      link: 10_github-and-remotes.html
    - name: Pulling and Collaborating
      link: 11_pulling-and-collaborating.html
  - section: Putting It All Together
    content:
    - name: 'Project: Version Control a Robot'
      link: 12_project-version-control-a-robot.html
    - name: Cheat Sheet and Next Steps
      link: 13_cheat-sheet-and-next-steps.html
---


## Your project's diary

Every commit you make is stored forever (or until you deliberately remove it). In this module we'll look at the three commands you use to explore that history:

* `git log`: the list of commits
* `git show`: what one commit changed
* `git diff`: what's changed but isn't committed yet

---

## git log

Run this in your `pico-bot` folder:

```bash
git log
```

```text
commit 8b1e4d07c2a9f35e1d6b0a4c7e8f9a2b3c4d5e6f (HEAD -> main)
Author: Ada Lovelace <ada@example.com>
Date:   Wed Nov 4 10:42:13 2026 +0000

    Set up the left motor pin

commit 3f9c2a1b7d5e4f6a8c9b0d1e2f3a4b5c6d7e8f9a
Author: Ada Lovelace <ada@example.com>
Date:   Wed Nov 4 10:31:02 2026 +0000

    Add start-up code that blinks the LED
```

The newest commit is at the top. Each one shows its full hash, the author, the date and the message. `HEAD -> main` marks where you are now: **HEAD** is Git's name for "the commit you've currently got checked out", and it's on the `main` branch.

If there are lots of commits, Git shows them a page at a time. Press `Space` to see more, and `q` to quit.

### A shorter log

Most of the time, you'll want the one-line version:

```bash
git log --oneline
```

```text
8b1e4d0 (HEAD -> main) Set up the left motor pin
3f9c2a1 Add start-up code that blinks the LED
```

You can use those short hashes anywhere Git asks for a commit.

### Other handy options

```bash
git log --oneline -5          # only the last 5 commits
git log --oneline main.py     # only commits that changed main.py
git log --oneline --graph --all   # draw the branches as a graph
```

The `--graph --all` version becomes really useful once you start using branches in module 8.

---

## git show

To see exactly what a commit changed, give `git show` its hash:

```bash
git show 8b1e4d0
```

```text
commit 8b1e4d07c2a9f35e1d6b0a4c7e8f9a2b3c4d5e6f (HEAD -> main)
Author: Ada Lovelace <ada@example.com>
Date:   Wed Nov 4 10:42:13 2026 +0000

    Set up the left motor pin

diff --git a/main.py b/main.py
index 1a2b3c4..5d6e7f8 100644
--- a/main.py
+++ b/main.py
@@ -3,6 +3,7 @@ from machine import Pin
 import time

 led = Pin("LED", Pin.OUT)
+left_motor = Pin(14, Pin.OUT)

 print("Pico-Bot starting up")
```

The bottom part is called a **diff**. Lines starting with `+` were added, lines starting with `-` were removed, and the others are shown for context. The `@@` line tells you roughly where in the file the change is.

`git show` on its own (no hash) shows the latest commit.

---

## git diff

`git diff` shows changes you **haven't committed yet**. Add a right motor to `main.py`:

```python
right_motor = Pin(15, Pin.OUT)
```

Then run:

```bash
git diff
```

```text
diff --git a/main.py b/main.py
index 5d6e7f8..9a0b1c2 100644
--- a/main.py
+++ b/main.py
@@ -4,6 +4,7 @@ import time

 led = Pin("LED", Pin.OUT)
 left_motor = Pin(14, Pin.OUT)
+right_motor = Pin(15, Pin.OUT)
```

Here's the catch: `git diff` compares your files with the **staging area**. Once you `git add` a change, plain `git diff` no longer shows it. To see what's staged and about to be committed, use:

```bash
git diff --staged
```

So the full picture is:

Command | Shows
--------|------
`git diff` | Changes you've made but not staged
`git diff --staged` | Changes that are staged and will go in the next commit
`git diff 3f9c2a1 8b1e4d0` | Differences between two commits
{:class="table table-striped"}

A great habit: run `git diff --staged` just before every commit, and read through what you're about to save.

Now commit the right motor:

```bash
git add main.py
git commit -m "Set up the right motor pin"
```

---

## Try it yourself

1. Run `git log --oneline` and count your commits.
2. Use `git show` on your very first commit.
3. Make a change, run `git diff`, then `git add` it and run `git diff` again. Where did the change go? Find it with `git diff --staged`.
4. Challenge: run `git log -p main.py`. What does `-p` add?

---

## Common issues

- **Problem**: The log fills the screen and you can't type
- **Solution**: Press `q` to quit.
- **Why**: Git shows long output in a *pager* called `less`, which waits for you to scroll.

- **Problem**: `git diff` shows nothing, but you know you changed something
- **Solution**: Try `git diff --staged`.
- **Why**: Once a change is staged, plain `git diff` doesn't show it.

- **Problem**: `fatal: your current branch 'main' does not have any commits yet`
- **Solution**: Make a commit first (module 4).
- **Why**: There's no history to show until there's at least one commit.

---

## Check your understanding

{% quiz %}
? Which command lists your commits, one per line?
- [ ] `git status --short`
- [x] `git log --oneline`
- [ ] `git show --short`
> `git log --oneline` shows the short hash and message of each commit, newest first.

? You staged a change with `git add`. Which command shows it?
- [ ] `git diff`
- [x] `git diff --staged`
- [ ] `git log`
> Plain `git diff` compares your files with the staging area, so staged changes only appear with `--staged`.

? In a diff, what does a line starting with `+` mean?
- [x] The line was added
- [ ] The line was removed
- [ ] The line is a comment
> `+` lines were added and `-` lines were removed.
{% endquiz %}

---
