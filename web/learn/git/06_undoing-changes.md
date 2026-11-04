---
layout: lesson
title: Undoing Changes
author: Kevin McAleer
type: page
cover: /learn/git/assets/cover.jpg
date: 2026-11-04
previous: 05_viewing-history.html
next: 07_ignoring-files.html
description: Throw away edits with git restore, unstage files, fix your last commit
  with --amend, and undo commits safely with git revert
percent: 49
duration: 6
quiz_total: 35
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


## Mistakes are fine

The whole point of Git is that mistakes stop being scary. Which command you need depends on **where** the mistake is:

Where's the mistake? | What you want | Command
---------------------|---------------|--------
Edited a file, not staged | Throw the edit away | `git restore <file>`
Staged something by accident | Unstage it, keep the edit | `git restore --staged <file>`
Last commit has a typo or missed a file | Fix the last commit | `git commit --amend`
A commit broke something | Undo it with a new commit | `git revert <hash>`
{:class="table table-striped"}

Let's try each one on `pico-bot`.

---

## Throw away an edit: git restore

Say you've been experimenting and made a mess of `main.py`. It isn't staged, and you just want it back the way it was at the last commit:

```bash
git restore main.py
```

The file instantly goes back to how it was in the last commit.

> ## This one is permanent
>
> `git restore <file>` throws your uncommitted edits away, and Git can't get them back because they were never committed. Check with `git diff` first. This is one of the very few ways to really lose work in Git.

To throw away all uncommitted edits in the whole project, use `git restore .`.

---

## Unstage a file: git restore --staged

You ran `git add .` and accidentally staged `notes.txt` along with `main.py`. To take it out of the staging area but **keep your edits**:

```bash
git restore --staged notes.txt
```

Now `git status` shows `notes.txt` as not staged, and `main.py` is still staged and ready to commit. This one is completely safe: nothing is lost.

---

## Fix the last commit: git commit --amend

You've just committed, then spotted a typo in the message, or realised you forgot to add a file. Amend lets you redo the last commit.

To change just the message:

```bash
git commit --amend -m "Set up the right motor pin"
```

To add a forgotten file to the last commit:

```bash
git add README.md
git commit --amend --no-edit
```

`--no-edit` keeps the existing message.

> ## Only amend commits you haven't shared
>
> Amending replaces the last commit with a new one (it gets a new hash). That's fine on your own computer, but if you've already pushed the commit to GitHub (module 10), other people may have the old one. For shared commits, use `git revert` instead.

---

## Undo a commit safely: git revert

Suppose a commit two days ago made the robot drive backwards. You don't want to rewrite history; you just want to undo that change. `git revert` makes a **new commit** that does the exact opposite of the old one:

```bash
git log --oneline
```

```text
c7d2e19 (HEAD -> main) Add README
a41f8b3 Reverse motor direction
5d0e7b2 Set up the right motor pin
8b1e4d0 Set up the left motor pin
3f9c2a1 Add start-up code that blinks the LED
```

```bash
git revert a41f8b3
```

Git opens your editor with a message like `Revert "Reverse motor direction"`. Save and close it, and look at the log again:

```text
e5a9c40 (HEAD -> main) Revert "Reverse motor direction"
c7d2e19 Add README
a41f8b3 Reverse motor direction
...
```

The bad change is undone, and the history still shows what happened and when. This is the safe way to undo, especially on shared branches.

---

## Going back in time to look around

Sometimes you just want to see your project as it was, for example to check whether an old version of the robot code worked. You can check out any commit:

```bash
git switch --detach 3f9c2a1
```

Your files now look exactly as they did in that commit. This is called a **detached HEAD** state, which simply means you're looking at a commit rather than a branch (`git status` will say `HEAD detached at 3f9c2a1`). Have a look around, then go back to the present with:

```bash
git switch main
```

---

## A word about git reset

You'll see `git reset` a lot online. It moves your branch back to an earlier commit. The gentlest version undoes the last commit but keeps all its changes staged, so nothing is lost:

```bash
git reset --soft HEAD~1
```

`HEAD~1` means "one commit before HEAD". Be careful with `git reset --hard`, which **throws away** commits and uncommitted changes. As a beginner, reach for `git revert` first.

---

## Try it yourself

1. Scribble some nonsense into `main.py`, then get rid of it with `git restore`.
2. Create `notes.txt`, `git add` it, then unstage it with `git restore --staged`.
3. Make a commit with a deliberate typo in the message, then fix it with `--amend`.
4. Make a commit that changes the LED blink to 10 times, then undo it with `git revert`. Check the log.

---

## Common issues

- **Problem**: `git revert` says there's a conflict
- **Solution**: Later commits changed the same lines. Fix the file as shown in module 9, then run `git add` and `git revert --continue`. Or give up with `git revert --abort`.
- **Why**: Git can't automatically undo a change if the lines have changed again since.

- **Problem**: `git status` says `HEAD detached at 3f9c2a1`
- **Solution**: Run `git switch main` to get back to your branch.
- **Why**: You checked out a commit directly rather than a branch. Commits you make here don't belong to any branch, so don't work here.

- **Problem**: You used `git reset --hard` and lost a commit
- **Solution**: Run `git reflog`, find the hash of the lost commit, and run `git reset --hard <hash>` to go back to it.
- **Why**: The reflog remembers everywhere HEAD has been recently, so lost commits can usually be rescued. Uncommitted changes can't.

---

## Check your understanding

{% quiz %}
? You staged `secret.txt` by mistake. How do you unstage it but keep the file?
- [ ] `git restore secret.txt`
- [x] `git restore --staged secret.txt`
- [ ] `git revert secret.txt`
> `--staged` only takes the file out of the staging area. Your edits stay.

? A commit you pushed to GitHub last week broke the robot. What's the safest way to undo it?
- [ ] `git commit --amend`
- [ ] `git reset --hard`
- [x] `git revert <hash>`
> Revert adds a new commit that undoes the old one, so it doesn't rewrite history other people may already have.

? Which of these can permanently lose work?
- [x] `git restore main.py` with uncommitted edits
- [ ] `git restore --staged main.py`
- [ ] `git revert HEAD`
> Uncommitted edits were never saved by Git, so once restored over they're gone.
{% endquiz %}

---
