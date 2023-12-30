---
title: Deep Learning with Python
description: Understand the basics of deep learning and explore how Python's TensorFlow and Keras libraries can be used for creating deep learning models.
layout: lesson
cover: /learn/python/assets/6.png
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
