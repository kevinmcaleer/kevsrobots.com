---
layout: course_overview
title: Building User Authentication for Static Sites with FastAPI
author: Kevin McAleer
cover: /learn/fastapi/assets/cover.png
date: 2024-03-28
description: 'Build a robot telemetry API with Python and FastAPI: a Pico or any robot
  posts sensor readings, a static Jekyll dashboard shows them, and only logged-in
  users can switch the robot''s LEDs. You will use Pydantic v2 models, SQLite, hashed
  passwords, signed tokens, CORS and a safe deployment.'
level: intermediate
course_duration: 1h 5m
lesson_count: 10
first_lesson: 00_intro.html
quiz_total: 27
outcomes:
- Build and run a FastAPI app with fastapi dev, and explain what uvicorn does
- Validate requests with Pydantic v2 models and Annotated parameters
- Store users and sensor readings in SQLite with a lifespan handler and a database
  dependency
- Register users, hash their passwords and issue signed login tokens
- Protect routes so only logged-in users, or only admins, can reach them
- Let a static Jekyll page call the API with CORS, and deploy with settings from the
  environment
prerequisites:
- Python basics, such as functions, dictionaries and imports
- Helpful but not required, the Pydantic course (Mastering Pydantic for Robust Data
  Validation)
- Helpful but not required, Create Databases with Python and SQLite3
you_need:
- A computer with Python 3.10 or newer and a terminal
- A text editor such as Thonny, VS Code or Snakie, and a web browser
- Internet access to install packages with pip
- Optional, a Raspberry Pi Pico W or other robot to act as the sensor
- Optional, a Jekyll site, or the Docker setup from the Jekyll course
navigation:
- name: Building User Authentication for Static Sites with FastAPI
- content:
  - section: Introduction to User Authentication
    content:
    - name: Introduction to FastAPI and User Authentication Basics
      link: 00_intro.html
    - name: Setting Up the Development Environment for FastAPI
      link: 02_setting_up_the_environment.html
  - section: FastAPI and User Authentication Basics
    content:
    - name: Routes, Parameters and Request Bodies
      link: 03_fastapi_intro.html
    - name: How User Authentication Works
      link: 04_user_authentication_flow.html
  - section: Building the Authentication API
    content:
    - name: Models, SQLite and Lifespan
      link: 05_creating_user_models.html
    - name: Registration and Login
      link: 06_implementing_registration_and_login.html
    - name: Protected Routes and User Management
      link: 07_user_management_apis.html
  - section: Integrating with a Jekyll Site
    content:
    - name: Connecting a Static Jekyll Site to the API
      link: 08_integrating_with_jekyll.html
    - name: Settings, Secrets and Deployment
      link: 09_deploying_the_solution.html
    - name: Summary and What's Next
      link: 10_summary.html
---
