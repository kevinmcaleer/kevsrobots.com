---
layout: course_overview
title: OctaPi
author: Kevin McAleer
cover: /learn/octapi/assets/octapi.jpg
date: 2024-02-18
description: Build a cluster of Raspberry Pis and use Python and dispy to share out
  the work of cracking an Enigma message. Start with py-enigma on one computer, then
  scale up.
level: intermediate
course_duration: 0h 40m
lesson_count: 7
first_lesson: 00_intro.html
quiz_total: 18
video: cIq2AEbYotA
outcomes:
- Encrypt and decrypt messages with the py-enigma package
- Count the settings in a brute force attack and use a crib to spot the answer
- Write a dispy client that submits jobs and collects results
- Set up Raspberry Pi nodes with fixed addresses and run dispynode on each
- Measure the speedup of a cluster against a single computer
prerequisites:
- Basic Python, including functions and loops
- Comfort typing commands in a terminal
- A rough idea of IP addresses
you_need:
- A computer with Python 3.11 or 3.12 (3.13 needs a small workaround)
- Two or more Raspberry Pi 4 or 5 boards with power supplies and microSD cards, for
  the full cluster
- An Ethernet switch and cables
- Raspberry Pi Imager and an SSH client on your computer
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
    - name: Brute force decryption
      link: 04_lesson04.html
  - section: Build OctaPi
    content:
    - name: DisPy
      link: 05_dispy.html
    - name: Cracking Enigma with OctaPi
      link: 06_octapi.html
---
