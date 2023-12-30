---
layout: lesson
title: Deep Learning with Python
author: Kevin McAleer
type: page
cover: /learn/python/assets/6.png
date: 2023-07-20
previous: 12_ml.html
next: 14_automation.html
description: Understand the basics of deep learning and explore how Python's TensorFlow
  and Keras libraries can be used for creating deep learning models.
percent: 65
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

Deep Learning is a subfield of machine learning that uses algorithms inspired by the structure and function of the brain's neural networks. Python provides libraries like TensorFlow and Keras which makes it a great language for deep learning. This lesson will introduce the basics of deep learning and how to use TensorFlow and Keras for creating deep learning models.

---

## Learning Objectives

- Understand what deep learning is and its relation to artificial neural networks.
- Learn how to use the TensorFlow and Keras libraries for deep learning.

---

### What is Deep Learning?

Deep learning is a machine learning technique that teaches computers to do what comes naturally to humans: learn by example. It is a key technology behind driverless cars, enabling them to recognize a stop sign, or to distinguish a pedestrian from a lamppost.

---

### Introduction to TensorFlow

TensorFlow is a free and open-source software library for machine learning and artificial intelligence. It can be used across a range of tasks but has a particular focus on training and inference of deep neural networks.

```python
import tensorflow as tf

# Create a constant tensor
hello = tf.constant('Hello, TensorFlow!')

# To access a Tensor value, call numpy().
print(hello.numpy())
```

---

### Introduction to Keras

Keras is a user-friendly neural network library written in Python. It is capable of running on top of TensorFlow. Keras makes it really for ML beginners to build and design a neural network.

```python
from keras.models import Sequential
from keras.layers import Dense

# Define the model
model = Sequential()

# Add model layers
model.add(Dense(units=64, activation='relu', input_dim=100))
model.add(Dense(units=10, activation='softmax'))

# Compile the model
model.compile(loss='categorical_crossentropy', optimizer='adam', metrics=['accuracy'])
```

---

## Summary

In this lesson, you've learned about the basics of deep learning and its relation to artificial neural networks. We introduced TensorFlow, a library for large-scale machine learning, and Keras, a high-level neural networks API that runs on top of TensorFlow. These powerful tools open up a wide range of possibilities for training deep learning models.

---
