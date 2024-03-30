---
layout: lesson
title: Setting Up the Development Environment for FastAPI
author: Kevin McAleer
type: page
cover: assets/2.png
date: 2024-03-28
previous: 01_course_overview.html
next: 03_fastapi_intro.html
description: This lesson walks through the process of setting up your development
  environment for building a FastAPI application, including installing Python, FastAPI,
  and related tools.
percent: 22
duration: 2
navigation:
- name: Building User Authentication for Static Sites with FastAPI
- content:
  - section: Introduction to User Authentication
    content:
    - name: Introduction to FastAPI and User Authentication Basics
      link: 01_course_overview.html
    - name: Setting Up the Development Environment for FastAPI
      link: 02_setting_up_the_environment.html
  - section: FastAPI and User Authentication Basics
    content:
    - name: Introduction to FastAPI
      link: 03_fastapi_intro.html
    - name: User Authentication Flow
      link: 04_user_authentication_flow.html
  - section: Building the Authentication API
    content:
    - name: Creating User Models with Pydantic and SQLAlchemy
      link: 05_creating_user_models.html
    - name: Implementing Registration and Login with FastAPI
      link: 06_implementing_registration_and_login.html
    - name: Securing Endpoints with JWT Tokens in FastAPI
      link: 07_user_management_apis.html
  - section: Integrating with a Jekyll Site
    content:
    - name: Integrating FastAPI Authentication with a Jekyll Static Site
      link: 08_integrating_with_jekyll.html
    - name: Deploying Your FastAPI Authentication System and Jekyll Site
      link: 09_deploying_the_solution.html
---


![Setup Environment cover image]({{ page.cover }}){:class="cover"}

## Introduction

Before diving into coding, it's essential to prepare a development environment that's robust, isolated, and capable of handling our project's needs. This setup ensures that you can develop and test your application with consistency and reliability.

## Installing Python

FastAPI requires Python 3.7+. If you haven't installed Python yet or are using an older version, download and install the latest Python version from [the official Python website](https://www.python.org/downloads/).

## Setting Up a Virtual Environment

A virtual environment is a self-contained directory that contains a Python installation for a particular version of Python, plus a number of additional packages. Using a virtual environment allows you to manage dependencies for different projects, avoiding conflicts between package versions.

To create a virtual environment, run the following commands in your terminal:

```bash
python -m venv fastapi-env
```

To activate the virtual environment, use:

- On Windows:

```bash
fastapi-env\Scripts\activate.bat
```

- On Unix or MacOS:

```bash
source fastapi-env/bin/activate
```

## Installing FastAPI and Uvicorn

With your virtual environment activated, install FastAPI and Uvicorn (an ASGI server for running your application) using pip:

```bash
pip install fastapi uvicorn
```

## Setting Up Your Project Directory

Create a new directory for your project and navigate into it:

```bash
mkdir fastapi_auth_project
cd fastapi_auth_project
```

Within this directory, create a new file named `main.py`. This file will serve as the entry point for our FastAPI application.

## Summary

You've now set up your development environment, including Python, a virtual environment, and necessary packages like FastAPI and Uvicorn. In the next lesson, we'll dive into the basics of FastAPI and start building our user authentication system.

## Additional Resources

- [Virtual Environments in Python](https://docs.python.org/3/tutorial/venv.html)
- [FastAPI Official Documentation](https://fastapi.tiangolo.com/)

## Lesson Assignment

Try installing another Python package within your virtual environment and see how it is isolated from the global Python installation. Reflect on how using virtual environments can benefit the development process, especially when working on multiple projects.

---
