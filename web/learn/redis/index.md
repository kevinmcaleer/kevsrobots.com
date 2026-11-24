---
layout: course_overview
title: Using Redis with Python
author: Kevin McAleer
cover: /learn/redis/assets/redis-cover.jpg
date: 2023-04-12
description: Learn how to use Redis with Python to share robot sensor readings, queue
  jobs, count visitors, rank scores and send live messages, then build a robot telemetry
  hub with a live dashboard.
level: intermediate
course_duration: 1h 9m
lesson_count: 12
first_lesson: 00_intro.html
quiz_total: 37
outcomes:
- Install Redis and talk to it from redis-cli and from Python
- Choose the right data type for latest readings, job queues, unique counts and leaderboards
- Group commands with pipelines and transactions, and handle Redis errors
- Send live messages between programs with pub/sub
- Make keys expire, and explain what RDB snapshots and the append-only file protect
- Build a small robot telemetry hub with a publisher, a job queue and a dashboard
prerequisites:
- Comfortable writing simple Python programs, such as Python for beginners
- Helpful but not required, the Docker course
you_need:
- A computer with Python 3 (a Raspberry Pi 4 or 5 works well)
- Redis installed on Linux or macOS, or Docker for Windows
- An internet connection to install Redis and the redis Python library
- No robot is required, the lessons use simulated sensor readings
navigation:
- name: Using Redis with Python
- content:
  - section: Introduction to Redis
    content:
    - name: Introduction to Redis
      link: 00_intro.html
    - name: Redis Basics
      link: 02_redis_basics.html
  - section: Working with data
    content:
    - name: Redis Data Types
      link: 03_data_types.html
    - name: Redis Commands and Transactions
      link: 04_commands_and_transactions.html
    - name: Redis Lists
      link: 05_redis_lists.html
  - section: Sets, Hashes and Sorted Sets
    content:
    - name: Redis Sets
      link: 06_redis_sets.html
    - name: Redis Hashes
      link: 07_redis_hashes.html
    - name: Redis Sorted Sets
      link: 08_redis_sorted_sets.html
  - section: Publish and Subscribe
    content:
    - name: Redis Pub/Sub
      link: 09_redis_pub_sub.html
  - section: Keeping data
    content:
    - name: Expiry and Persistence
      link: 10_expiry_and_persistence.html
  - section: Project and Conclusion
    content:
    - name: 'Project: A Robot Telemetry Hub'
      link: 11_robot_telemetry_hub.html
    - name: Conclusion and Additional Resources
      link: 12_summary.html
---
