---
title: Your First Repository
description: >-
    Create a Git repository with git init, look inside the .git folder, and learn to read git status
layout: lesson
type: page
---

## Make a project folder

Let's create the project we'll use for the rest of the course: the code for a small two-wheeled robot. Open a terminal and run:

```bash
mkdir pico-bot
cd pico-bot
```

`mkdir` makes a new folder and `cd` moves into it. Everything from now on happens inside `pico-bot`.

---

## Turn it into a repository

To tell Git to start tracking this folder, run:

```bash
git init
```

```text
Initialized empty Git repository in /home/kev/pico-bot/.git/
```

That's it, `pico-bot` is now a Git repository. Git has created a hidden folder called `.git` inside it. List everything, including hidden files, to see it:

```bash
ls -a
```

```text
.  ..  .git
```

The `.git` folder is where Git keeps every commit, every branch and all its settings for this project. You never need to edit anything inside it. Just know that **this folder is the repository**: if you delete it, Git forgets the whole history (your normal files stay where they are).

> ## One repository per project
>
> Run `git init` in the top folder of each project, not in your home folder or your whole Documents folder. If you accidentally run it in the wrong place, just delete the `.git` folder that it created.

---

## Ask Git what's going on

The command you'll use more than any other is `git status`. It tells you what branch you're on and what has changed since your last commit. Try it now:

```bash
git status
```

```text
On branch main

No commits yet

nothing to commit (create/copy files and use "git add" to track)
```

Nothing to see yet, because the folder is empty. Let's add a file. Create a file called `main.py` in the `pico-bot` folder with your editor, and put this in it:

```python
# Pico-Bot: a simple two-wheeled robot
from machine import Pin
import time

led = Pin("LED", Pin.OUT)

print("Pico-Bot starting up")
for _ in range(3):
    led.toggle()
    time.sleep(0.5)
```

Save it and run `git status` again:

```text
On branch main

No commits yet

Untracked files:
  (use "git add <file>..." to include in what will be committed)
	main.py

nothing added to commit but untracked files present (use "git add" to track)
```

Git has noticed `main.py`, but it's **untracked**: Git can see the file, but isn't saving its history yet. Git never starts tracking a file until you tell it to. In the next module you'll do exactly that, and make your first commit.

---

## Reading git status

`git status` is your best friend. Get into the habit of running it before and after every other Git command. It tells you:

* **Which branch** you're on
* **Untracked files**: new files Git isn't tracking yet
* **Changes not staged for commit**: tracked files you've edited
* **Changes to be committed**: changes you've marked to go in the next commit

Even better, it usually tells you which command to run next, in the brackets.

If you prefer something shorter, try:

```bash
git status --short
```

```text
?? main.py
```

`??` means untracked. You'll meet other letters such as `M` (modified) and `A` (added) soon.

---

## Getting a project that already exists

There's a second way to get a repository: copy someone else's with `git clone`. You'll use that in module 10, once we've covered GitHub. For now, `git init` is all you need.

---

## Try it yourself

1. Create the `pico-bot` folder, run `git init`, and check `ls -a` shows the `.git` folder.
2. Create `main.py` and run `git status`.
3. Create a second file called `README.md` containing `# Pico-Bot`. Run `git status --short`. How many untracked files are there now?

---

## Common issues

- **Problem**: `fatal: not a git repository (or any of the parent directories): .git`
- **Solution**: `cd` into your project folder, or run `git init` if it isn't a repository yet.
- **Why**: Git commands only work inside a repository. Git looks for a `.git` folder in the current folder and every folder above it.

- **Problem**: `git status` lists hundreds of files from all over your computer
- **Solution**: You've run `git init` in your home folder. Delete the `.git` folder there (`rm -rf ~/.git`, double-check the path first!) and run `git init` in the project folder instead.
- **Why**: Git tracks everything inside the folder where you ran `git init`.

- **Problem**: You can't see the `.git` folder
- **Solution**: Use `ls -a`, or turn on hidden files in your file manager (`Cmd + Shift + .` on a Mac).
- **Why**: Names starting with a dot are hidden by default.

---

## Check your understanding

{% quiz %}
? What does `git init` do?
- [ ] Uploads the folder to GitHub
- [x] Creates a `.git` folder so Git starts tracking this folder as a repository
- [ ] Makes the first commit
> `git init` creates an empty repository. No files are saved until you add and commit them.

? `git status` lists `main.py` under "Untracked files". What does that mean?
- [x] Git can see the file but isn't saving its history yet
- [ ] The file has been deleted
- [ ] The file has been committed
> Git never tracks a new file until you tell it to with `git add`.
{% endquiz %}

---
