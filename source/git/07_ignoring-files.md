---
title: Ignoring Files
description: >-
    Keep junk files, build output and secrets out of your repository with .gitignore
layout: lesson
type: page
---

## Not everything belongs in Git

Projects collect files you never want to commit:

* **Generated files**: Python's `__pycache__` folders, compiled firmware, build folders
* **System junk**: `.DS_Store` on a Mac, `Thumbs.db` on Windows
* **Editor settings**: some editors save personal settings in the project folder
* **Secrets**: Wi-Fi passwords, API keys and tokens

Secrets are the important one. If you commit your Wi-Fi password and push to GitHub, it's public, and it stays in the history even if you delete the file later.

---

## The .gitignore file

You tell Git which files to ignore with a file called `.gitignore` (note the dot at the start) in the top folder of your repository. Each line is a pattern. Ignored files don't show up in `git status`, and `git add .` skips them.

Let's say our robot connects to Wi-Fi, and we keep the password in `secrets.py`:

```python
# secrets.py - never commit this!
WIFI_SSID = "MyNetwork"
WIFI_PASSWORD = "hunter2"
```

Run `git status --short` and Git wants to track it:

```text
?? secrets.py
```

Create `.gitignore` with this in it:

```text
# Passwords and keys
secrets.py
.env

# Python
__pycache__/
*.pyc

# System junk
.DS_Store
Thumbs.db
```

Run `git status --short` again:

```text
?? .gitignore
```

`secrets.py` has vanished from the list. Git is ignoring it. Notice that `.gitignore` itself shows up: you **should** commit it, so everyone working on the project ignores the same files.

```bash
git add .gitignore
git commit -m "Ignore secrets and Python cache files"
```

---

## Pattern rules

Pattern | What it ignores
--------|----------------
`secrets.py` | Any file called `secrets.py`, in any folder
`*.log` | Every file ending in `.log`
`build/` | Any folder called `build`, and everything in it
`/todo.txt` | `todo.txt` in the top folder only, not in sub-folders
`docs/*.pdf` | PDFs directly in the `docs` folder
`!keep.log` | Don't ignore `keep.log`, even though `*.log` matched it
`# comment` | Lines starting with `#` are comments
{:class="table table-striped"}

You don't have to write these from scratch. GitHub keeps a collection of ready-made ones at [github.com/github/gitignore](https://github.com/github/gitignore). The `Python.gitignore` one is a good starting point for MicroPython projects.

---

## Share an example instead of the secret

Other people (and future you) still need to know what goes in `secrets.py`. A common trick is to commit an example file with fake values:

```python
# secrets_example.py
# Copy this file to secrets.py and fill in your own details.
WIFI_SSID = "your-network-name"
WIFI_PASSWORD = "your-password"
```

`secrets_example.py` doesn't match the pattern `secrets.py`, so it gets committed.

---

## Already committed it?

`.gitignore` only affects files Git isn't already tracking. If you committed a file before adding it to `.gitignore`, Git keeps tracking it. To stop tracking it but keep the file on your computer:

```bash
git rm --cached secrets.py
git commit -m "Stop tracking secrets.py"
```

> ## Leaked a real password?
>
> If you've already pushed a password or key to GitHub, **change the password or revoke the key straight away**. Removing the file in a new commit doesn't remove it from the history, and bots scan GitHub for leaked keys within minutes.

---

## Checking why a file is ignored

If a file is mysteriously missing from `git status`, ask Git which rule is ignoring it:

```bash
git check-ignore -v secrets.py
```

```text
.gitignore:2:secrets.py	secrets.py
```

That means line 2 of `.gitignore` matched it.

---

## Try it yourself

1. Create `secrets.py` and a `.gitignore` that ignores it. Check `git status`.
2. Create a file called `debug.log`, and add a pattern to ignore all `.log` files.
3. Commit your `.gitignore` and a `secrets_example.py`.
4. Challenge: create a folder called `build` with a file in it. Ignore the whole folder with one line.

---

## Common issues

- **Problem**: A file is in `.gitignore` but Git still shows it as modified
- **Solution**: Run `git rm --cached <file>` and commit.
- **Why**: `.gitignore` doesn't apply to files that are already tracked.

- **Problem**: `.gitignore` doesn't seem to do anything
- **Solution**: Check the file is called exactly `.gitignore` (not `gitignore.txt`) and is in the top folder of the repository.
- **Why**: Windows Notepad sometimes adds `.txt` to the end of file names.

- **Problem**: An empty folder doesn't show up in Git at all
- **Solution**: Put a file in it. Many people add an empty file called `.gitkeep`.
- **Why**: Git tracks files, not folders. An empty folder has nothing to track.

---

## Check your understanding

{% quiz %}
? Should you commit the `.gitignore` file itself?
- [x] Yes, so everyone on the project ignores the same files
- [ ] No, it should be ignored too
- [ ] Only if the repository is private
> `.gitignore` is part of the project's setup, so commit it.

? You committed `secrets.py` yesterday, then added it to `.gitignore` today. What happens?
- [ ] Git deletes it from the history
- [x] Git keeps tracking it until you run `git rm --cached secrets.py`
- [ ] Git ignores it from now on automatically
> Ignore rules only apply to untracked files. If the file contains a real password that's been pushed, change the password too.

? Which pattern ignores every `.log` file?
- [ ] `log`
- [x] `*.log`
- [ ] `/log/`
> `*` matches any characters, so `*.log` matches every name ending in `.log`.
{% endquiz %}

---
