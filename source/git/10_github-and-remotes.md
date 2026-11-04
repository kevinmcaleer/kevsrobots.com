---
title: GitHub and Remotes
description: >-
    Put your repository on GitHub, push your commits, sign in safely, and clone repositories onto other computers
layout: lesson
type: page
---

## What's a remote?

So far your repository only exists on your computer. A **remote** is a copy of it somewhere else, usually on GitHub. Having one means:

* Your work is **backed up**: if your laptop dies, your project doesn't
* You can **share** it: anyone can see and download your robot code
* You can work on **more than one computer**: push from your laptop, pull onto your Raspberry Pi

You **push** your commits up to the remote, and **pull** new commits down from it.

```mermaid
flowchart LR
    L["Your computer<br/>(local repository)"] -- "git push" --> G["GitHub<br/>(remote: origin)"]
    G -- "git pull" --> L
    G -- "git clone" --> P["Raspberry Pi<br/>(another copy)"]
```

---

## Create a repository on GitHub

1. Sign in to [github.com](https://github.com) (sign up for a free account if you don't have one).
2. Click the **+** at the top right, then **New repository**.
3. Name it `pico-bot`, and choose **Public** or **Private**.
4. **Don't** tick "Add a README", ".gitignore" or "license". You already have a repository with commits, so you want GitHub's copy to start empty.
5. Click **Create repository**.

GitHub shows you a page with the address of your new repository. It looks like this:

```text
https://github.com/your-username/pico-bot.git
```

---

## Connect your repository to GitHub

Back in your `pico-bot` folder, tell Git about the remote. By convention, the main remote is called `origin`:

```bash
git remote add origin https://github.com/your-username/pico-bot.git
```

Check it worked:

```bash
git remote -v
```

```text
origin	https://github.com/your-username/pico-bot.git (fetch)
origin	https://github.com/your-username/pico-bot.git (push)
```

---

## Signing in to GitHub

GitHub doesn't accept your normal password from the command line. You need a safer way to sign in. Pick one:

* **Windows**: Git Credential Manager (installed with Git for Windows) pops up a browser window the first time you push. Sign in and you're done.
* **macOS, Linux and Raspberry Pi**: the easiest option is the [GitHub CLI](https://cli.github.com). Install it (`brew install gh` on a Mac, or follow the instructions on the website for Linux and Raspberry Pi OS), then run `gh auth login`. Choose **GitHub.com**, then **HTTPS**, and say **Yes** when it asks to authenticate Git with your GitHub credentials.
* **SSH keys** are another popular option. GitHub's guide to [connecting with SSH](https://docs.github.com/en/authentication/connecting-to-github-with-ssh) walks you through it. If you use SSH, your remote address starts `git@github.com:` instead of `https://`.

You only need to do this once per computer.

---

## Push your commits

Now send your commits to GitHub:

```bash
git push -u origin main
```

```text
Enumerating objects: 18, done.
Counting objects: 100% (18/18), done.
Writing objects: 100% (18/18), 2.10 KiB | 2.10 MiB/s, done.
To https://github.com/your-username/pico-bot.git
 * [new branch]      main -> main
branch 'main' set up to track 'origin/main'.
```

`-u` (short for `--set-upstream`) links your local `main` to `main` on `origin`. You only need it the first time. After that, just run:

```bash
git push
```

Refresh your repository page on GitHub. Your files are there, and if you click **commits** you'll see your whole history, every message you've written.

Pushing a new branch works the same way: `git push -u origin obstacle-avoidance`.

---

## Clone a repository

To get a copy of a repository onto another computer, such as your Raspberry Pi, use `git clone`:

```bash
git clone https://github.com/your-username/pico-bot.git
cd pico-bot
```

This downloads the whole repository, every commit and branch, into a new `pico-bot` folder, and sets up `origin` automatically. It works for anyone's public repository, so it's also how you download other people's projects:

```bash
git clone https://github.com/kevinmcaleer/smars.git
```

---

## Try it yourself

1. Create an empty `pico-bot` repository on GitHub and push your local one to it.
2. Make a change to `README.md`, commit it, and `git push`. Refresh GitHub to see it.
3. Clone your repository into a different folder (or onto a Raspberry Pi) with `git clone`.
4. Run `git status` after pushing. What does it say about `origin/main`?

---

## Common issues

- **Problem**: `remote: Support for password authentication was removed`
- **Solution**: Sign in with Git Credential Manager, `gh auth login`, or SSH, as described above.
- **Why**: GitHub stopped accepting account passwords for Git in 2021, for security.

- **Problem**: `error: failed to push some refs ... Updates were rejected because the remote contains work that you do not have locally`
- **Solution**: Run `git pull` first, then `git push` (see the next module).
- **Why**: Someone (perhaps you, on GitHub's website) has pushed commits you don't have. Git won't let you overwrite them.

- **Problem**: `error: src refspec main does not match any`
- **Solution**: Make at least one commit, and check your branch name with `git branch`. If it's called `master`, push with `git push -u origin master`, or rename it with `git branch -m main`.
- **Why**: You can only push a branch that exists and has commits.

- **Problem**: `error: remote origin already exists`
- **Solution**: Change its address with `git remote set-url origin <new-url>`.
- **Why**: You've already added a remote called `origin`.

---

## Check your understanding

{% quiz %}
? What is `origin`?
- [ ] The first commit in a repository
- [x] The usual name for the main remote repository
- [ ] The `main` branch on your computer
> `origin` is just a nickname for the remote's address, set by `git remote add` or `git clone`.

? Why use `-u` the first time you run `git push -u origin main`?
- [x] It links your local branch to the remote one, so later you can just type `git push`
- [ ] It uploads files faster
- [ ] It makes the repository public
> After the first push with `-u`, Git remembers where `main` should push to and pull from.

? Which command downloads a full copy of a repository from GitHub?
- [ ] `git pull`
- [ ] `git remote add`
- [x] `git clone`
> `git clone` copies the whole repository, history and all, and sets up `origin` for you.
{% endquiz %}

---
