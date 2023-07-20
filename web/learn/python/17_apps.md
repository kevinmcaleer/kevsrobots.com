---
layout: lesson
title: 'Project: Building a Simple Python Application'
author: Kevin McAleer
type: page
cover: assets/3.png
previous: 16_advanced.html
next: 18_tips_and_tricks.html
description: Put everything you've learned to use by building a simple Python application.
  This project will provide hands-on experience with Python programming.
percent: 85
duration: 2
navigation:
- name: Python for beginners
- content:
  - section: Introduction
    content:
    - name: Introduction to Python Programming
      link: 01_intro.html
    - name: Python Basics
      link: 02_basics.html
    - name: Python Data Structures
      link: 03_data_structures.html
    - name: Control Flow in Python
      link: 04_flow.html
    - name: Working with Files in Python
      link: 05_files.html
    - name: Error Handling and Exceptions in Python
      link: 06_errors.html
    - name: Python Libraries and Modules
      link: 07_libraries.html
    - name: Python Object-Oriented Programming (OOP)
      link: 08_oop.html
    - name: Working with Data in Python
      link: 09_data.html
  - section: Real world Python
    content:
    - name: Web Scraping with Python
      link: 10_webscraping.html
    - name: Data Visualization
      link: 11_data_visualisation.html
    - name: Machine Learning with Python
      link: 12_ml.html
    - name: Deep Learning with Python
      link: 13_deep_learning.html
    - name: Python for Automating Tasks
      link: 14_automation.html
    - name: Python and Databases
      link: 15_databases.html
    - name: Advanced Python Topics
      link: 16_advanced.html
    - name: 'Project: Building a Simple Python Application'
      link: 17_apps.html
    - name: 'Bonus: Python Tips, Tricks and Best Practices'
      link: 18_tips_and_tricks.html
---


![cover image]({{page.cover}}){:class="cover"}

## Introduction

In the past few lessons, we've learned about various aspects of Python programming. Now it's time to put all those concepts together and build a simple Python application. This will provide you with hands-on experience and help cement your Python knowledge.

---

## Learning Objectives

- Understand how different Python concepts fit together in a real-world application.
- Gain experience in problem-solving and Python programming by building an application from scratch.

---

### Project Overview: A Simple Quiz Application

We will build a simple quiz application. The application will do the following:

- Present a multiple-choice quiz to the user.
- Score the user's responses.
- Display the final score to the user at the end.

```python
class Question:
   def __init__(self, prompt, answer):
       self.prompt = prompt
       self.answer = answer

question_prompts = [
   "What color are apples?\n(a) Red/Green\n(b) Purple\n(c) Orange\n",
   "What color are Bananas?\n(a) Teal\n(b) Magenta\n(c) Yellow\n",
   "What color are strawberries?\n(a) Yellow\n(b) Red\n(c) Blue\n",
]

questions = [
   Question(question_prompts[0], "a"),
   Question(question_prompts[1], "c"),
   Question(question_prompts[2], "b"),
]

def run_quiz(questions):
   score = 0
   for question in questions:
       answer = input(question.prompt)
       if answer == question.answer:
           score += 1
   print(f"You got {score}/{len(questions)} correct")

run_quiz(questions)
```

---

## Summary

In this lesson, you've put the Python concepts you've learned to use by building a simple quiz application. This kind of hands-on experience is crucial in building your confidence and abilities as a Python programmer. Keep building more projects to improve and to keep your Python skills sharp.

---
