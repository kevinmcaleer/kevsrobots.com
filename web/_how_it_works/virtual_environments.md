---
layout: blog
title: Python Virtual Environments
short_title: How it works - Python Virtual Environments
short_description: Learn about Python Virtual Environments
date: 2023-07-12
author: Kevin McAleer
excerpt: 
cover: /assets/img/how_it_works/virtual_environments.png
tags:
 - Python
 - Virtual Environment
---

A Python `virtual environment` is a self-contained "bubble" where you can install and manage Python packages (which are essentially libraries or tools written in Python that you can use in your own projects) without affecting other projects on your computer.

![](/assets/img/how_it_works/virtual_environments.png){:class="img-fluid w-100"}

---

When you're working on a Python project, you might need to use certain versions of Python packages. However, if you were to install those packages globally (i.e., for the entire system), you might run into issues where one project needs one version of a package, and another project needs a different version.

![](/assets/img/how_it_works/virtual_environments03.jpg){:class="img-fluid w-100"}

To prevent these conflicts, you can use a Python virtual environment, which allows you to isolate the Python interpreter, libraries, and scripts used in a specific project. This means you can have multiple environments on your computer, each with different versions of Python and its packages, without them interfering with each other.

Think of it as a sandbox, where you can play around with different packages and their versions without worrying about messing up your other sandboxes.

You create a virtual environment using tools like `venv` (which is built into Python itself) or `virtualenv`, `conda`, `pipenv`, etc. After creating a virtual environment, you can then "activate" it, which basically means you're telling your computer to use the Python interpreter and packages from that environment, rather than the global Python interpreter and packages.

---

## Create a Virtual Environment

![](/assets/img/how_it_works/virtual_environments04.jpg){:class="img-fluid w-100"}

To create a Virtual Environment, type the following into your terminal:

``` bash
python3 -m venv venv
```

---

## Activate the environment

To activate a Virtual Environment, type the following into your terminal:

![](/assets/img/how_it_works/virtual_environments05.jpg){:class="img-fluid w-100"}

``` bash
source venv/bin/activate
```

---

## Deactivate the environment

To deactivate a Virtual Environment, type the following into your terminal:

``` bash
deactivate
```

---

Using virtual environments is considered a best practice in Python development, because it helps keep your projects organized and prevents issues with package versions and dependencies.

---
