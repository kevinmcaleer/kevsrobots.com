---
layout: lesson
title: DisPy
author: Kevin McAleer
type: page
cover: /learn/octapi/assets/octapi.jpg
date: 2024-02-18
previous: 04_lesson04.html
next: 06_octapi.html
description: learn how to use DisPy to parallelize your Python code across multiple
  machines
percent: 84
duration: 3
navigation:
- name: OctaPi
- content:
  - section: Overview
    content:
    - name: Introduction
      link: 00_intro.html
  - section: Introduction to Enigma
    content:
    - name: Encrypt
      link: 01_lesson01.html
    - name: Decrypting a message
      link: 02_lesson02.html
  - section: Bruteforce attack
    content:
    - name: Encrypt longer messages
      link: 03_lesson03.html
    - name: Bruteforce descryption
      link: 04_lesson04.html
  - section: Build OctaPi
    content:
    - name: DisPy
      link: 05_dispy.html
    - name: Cracking Enigma with OctaPi
      link: 06_octapi.html
---


## What is DisPy?

`DisPy` is a Python package that allows you to distribute and parallelize your Python code across multiple machines. It is similar to the `multiprocessing` module, but it allows you to run your code on multiple machines instead of just multiple cores on a single machine.

---

## How to install DisPy

To install DisPy, you can use pip:

```bash
pip install dispy
```

---

## Benefits of DisPy

There are several benefits to using DisPy to parallelize your Python code:

- **Speed**: By distributing your code across multiple machines, you can run it much faster than if you were running it on a single machine.

- **Scalability**: DisPy allows you to easily scale your code to run on as many machines as you need. This makes it easy to take advantage of the resources available to you, and to run your code on a large scale.

- **Fault tolerance**: DisPy is designed to be fault-tolerant, so if one of the machines in your cluster fails, your code will continue to run on the remaining machines.

- **Ease of use**: DisPy is easy to use, and it integrates well with Python's built-in multiprocessing module. This makes it easy to parallelize your code without having to learn a lot of new concepts.

---

Running DisPy on a Cluster of Raspberry Pi's is a great way to learn about parallel computing. You can use DisPy to distribute your Python code across multiple machines, and see how it runs in parallel. This can help you to understand the benefits and challenges of parallel computing, and how to write code that can take advantage of it.

---

## How to use DisPy

To use DisPy, you first need to create a cluster of machines that you want to run your code on. You can do this by creating an instance of the `JobCluster` class and passing in the function that you want to run, and the IP addresses of the machines that you want to run it on. Here's an example:

```python
import dispy

nodes = ['192.168.2.*','192.168.1.*']

def add_number(num):
    result = num + 1
    return result

cluster = dispy.JobCluster(add_number, nodes=nodes, loglevel=dispy.logger.DEBUG)

jobs = []
id=1
for n in range(1,100):
    job = cluster.submit(n)
    job.id = id # Associate an ID to the job
    jobs.append(job)
    id += 1   # Next job

print( "Waiting..." )
cluster.wait()

for job in jobs:
    print(f"job {job.id} result is: {job.result}")
```