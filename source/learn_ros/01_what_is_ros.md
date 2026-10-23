---
layout: lesson
title: What is ROS
type: page
description: Learn about ROS, the different versions and what it does

---

## What is ROS?

Here is the definition from the creators of ROS - <https://ros.org/>

> *"The Robot Operating System (ROS) is a set of software libraries and tools that help you build robot applications. From drivers to state-of-the-art algorithms, and with powerful developer tools, ROS has what you need for your next robotics project. And it's all open source."*

---

## Which version should I use

There are two versions of ROS:

* **ROS 1** - The original version of ROS. Its last release, Noetic, reached end of life in May 2025.
* **ROS 2** - The current version of ROS, and what we will be using in this course.

---

## Releases & Distros

ROS 2 has a new release (called a *distro*) every May. The releases in even years are Long Term Support (LTS) releases with five years of updates. This course uses **Humble Hawksbill**, which is supported until May 2027 and is what the example Docker files are built on. If you're starting a new project, check the [list of ROS 2 distributions](https://docs.ros.org/en/rolling/Releases.html) for the newest LTS release; the code in this course works the same way.

Distro              |     Release date |   End Of Life date
:-------------------|-----------------:|------------------:
Jazzy Jalisco (LTS) |     May 23, 2024 |           May 2029
Iron Irwini         |     May 23, 2023 |      December 2024
**Humble Hawksbill (LTS)** | **May 23, 2022** | **May 2027**
Foxy Fitzroy        |     June 5, 2020 |           May 2023
{:class="table table-striped"}

---

## Programming languages

ROS2 Supports both `C++` and `Python` out of the box (and I'm sure there are other languages adapted to use it too), however for this course we will be focusing on `Python`.

---

## Check your understanding

{% quiz %}
? Which ROS 2 distro does this course use?
- [ ] Foxy Fitzroy
- [x] Humble Hawksbill
- [ ] Iron Irwini
> Humble is a Long Term Support release, supported until May 2027.

? Which two programming languages does ROS 2 support out of the box?
- [x] C++ and Python
- [ ] Java and JavaScript
- [ ] Rust and Go
{% endquiz %}
