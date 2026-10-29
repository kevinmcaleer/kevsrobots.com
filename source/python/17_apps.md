---
title: "Project: Building a Simple Python Application"
description: >-
    Put everything you've learned to use by building a simple Python application. This project will provide hands-on experience with Python programming.
layout: lesson
cover: /learn/python/assets/3.png
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
       answer = input(question.prompt).strip().lower()
       if answer == question.answer:
           score += 1
   print(f"You got {score}/{len(questions)} correct")

run_quiz(questions)
```

### How it works

- The `Question` class bundles each prompt with its correct answer
- `run_quiz()` loops through the questions, asks each one with `input()` and adds a point for each right answer
- `.strip().lower()` tidies up the answer, so `" A "` still counts as `a`

---

## Try it yourself

- Add three more questions of your own
- Load the questions from a text file, using what you learned in the files lesson
- Tell the player the right answer when they get one wrong

---

## Summary

In this lesson, you've put the Python concepts you've learned to use by building a simple quiz application. This kind of hands-on experience is crucial in building your confidence and abilities as a Python programmer. Keep building more projects to improve and to keep your Python skills sharp.

---
