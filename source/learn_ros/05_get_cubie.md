---
layout: lesson
title: Get Cubie-1 files
type: page
description: Download the Cubie-1 example files
date_updated: 2026-11-24
cover: /learn/learn_ros/assets/banners/05_get_cubie.jpg
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to download the Cubie-1 example files with `git` and say what the folders inside are for.

## Example files

Cubie-1 is the robot this series is building towards. Its repository holds the files we need to follow along: the `dockerfile` that describes our ROS 2 container, a `docker-compose.yml` that says how to run it, and a ready-made `docker-full` setup that includes RViz2 for the later lessons. Grabbing these now saves a lot of typing.

## Get Cubie-1 files

* **Clone the repository** - from the terminal, in your home folder, type:

```bash
cd ~
git clone https://github.com/kevinmcaleer/cubie-1
```

`git clone` copies the whole project, including its history, into a new `cubie-1` folder. Look inside:

```bash
ls cubie-1
```

You should see folders called `docker` and `docker-full`. The first builds the plain ROS 2 container we use for the next few lessons. The second also installs the desktop tools, including RViz2.

```mermaid
flowchart LR
    R[GitHub: cubie-1] -- git clone --> H[~/cubie-1 on the Pi]
    H --> A[docker/<br/>core ROS 2]
    H --> B[docker-full/<br/>ROS 2 + RViz2]
    H -. shared into container as .-> C[/ros2]
```

Later we'll map this folder into the container, so the files you write in the container are also stored safely on the Pi's SD card.

---

## Try it Yourself

1. Use `ls ~/cubie-1/docker` to list the files inside the `docker` folder. Which one is the `dockerfile`?
2. Use `cat` to read the `docker-compose.yml`. Can you guess what each line does?
3. Challenge: run `git log --oneline | head` inside `cubie-1`. How many changes has the project had?

## Common Issues

- **Problem**: `git: command not found`.
- **Solution**: Install it with `sudo apt install git -y`, then clone again.
- **Why**: Git isn't always included on a fresh install.

- **Problem**: `destination path 'cubie-1' already exists and is not an empty directory`.
- **Solution**: You've already cloned it. Use `cd cubie-1` and `git pull` to update it instead.
- **Why**: `git clone` refuses to overwrite an existing folder.

## Check your understanding

{% quiz %}
? What does `git clone` do?
- [ ] Deletes a repository
- [x] Copies a repository, with its history, to your machine
- [ ] Builds a Docker image
> It makes a local copy that you can read, edit and update.

? Which folder holds the version of the container that includes RViz2?
- [ ] `docker`
- [x] `docker-full`
- [ ] `assets`
> `docker-full` also installs the desktop tools, as the lesson says.

? Why clone Cubie-1 into the home folder?
- [x] Later commands refer to `~/cubie-1`
- [ ] Git only works in the home folder
- [ ] Docker requires it
{% endquiz %}
