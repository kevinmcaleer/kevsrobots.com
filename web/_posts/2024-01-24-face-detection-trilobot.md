---
title: Face Detection
description: Learn how to set up face detection on your companion robot using Raspberry Pi and OpenCV.
layout: project
date: 2024-01-21
cover: /assets/img/blog/trilobot/trilobot.jpg
excerpt: >-
    Today's session aims to help you set up your raspberry PI robot to perform face detection, change lights based on the faces it sees, and give a brief demo of how it works.
author: Kevin McAleer
difficulty: beginner
groups:
    - raspberrypi
    - robots
    - ai
tags:
    - pimoroni
    - raspberry_pi
    - trilobot
    - opencv
    - face_detection
    - python
videos:
    - J8OI8z_BWtk
---

## Introduction

Hello Robotics Makers! I hope you are having a fantastic day. Ever wondered how to equip your Trilobot or any Raspberry Pi robot with a camera module to detect faces using OpenCV? If yes, then this article could be the guide you've been searching for! I'll break it down for you, and trust me, it's simpler than you think!

My name is Kevin, come with me as we build robots, bring them to life with code, and have loads of fun along the way. Today, the journey leads us to explore face detection using our little Trilobot.

---

## Face Detection with Companion Robots

Companion robots are interactive robots designed to assist, entertain, and interact with humans. One classic feature a companion robot can possess is face detection. This feature enables the robot to change its lights based on if it detects a face, enhancing its interaction with humans.

Today's session aims to help you set up your raspberry PI robot to perform face detection, change lights based on the faces it sees, and give a brief demo of how it works.

---

## Setting Up the Raspberry Pi

Setting up your Raspberry PI is straightforward.

1. We need to first prepare our Raspberry PI by performing

    `sudo apt update` to download the software's latest patches

1. Similarly, running

    `sudo apt upgrade` will install all the latest software versions.

Bear in mind that you may need to adjust to a legacy camera option if you do not see it available.

---

## Installing Open CV And PI Camera

Now, let's install the necessary libraries. OpenCV (Open Source Computer Vision Library) offers functionalities to help us use our PI camera. However, installing Open CV can throw errors, hence we recommend using

1. `sudo apt install python3-opencv` for a hurdle-free installation.

    Next on our setup list is installing PI camera. While it should automatically install when you enable your Raspberry PI camera, it's recommended to install it manually just in case.

1. To install the Picamera Python library,

    `sudo pip install picamera` accomplishes this neatly.

---

## Implementing Open CV

OpenCV is a library that enables us to carry out a variety of operations with simplicity, spanning several languages, Python included. Today we'll be using picamera to take a picture and then utilize OpenCV to detect how many faces are in the image. It could be zero, one, or even more. We'll also change the lights on our Trilobot based on the number of detected faces.

We can take this one step further and write out a file with a bounding box around the detected faces.

---

## Code on GitHub

You can find the code for this tutorial here: <https://www.github.com/kevinmcaleer/smarslab> within the 'non-smarslab' folder.

---

## Unpacking Open CV and Harr Cascade

Typically, Harr Cascade is unfamiliar to many. Harr is a precise mathematical sequence of rescale square shape functions that form a wavelet family.

Harr Cascade models are pre-configured models available for us. Our focus today lies on the Harr cascade front face default XML file and how we can use it for face detection.

When we gauge that pre-configured model's value against our picture data, it searches for features that represent a face and finds the sum of pixel values in the area. It finally tells us if the features are present in the region.

---

## Conclusion

This fascinating process outlines how faces are detected. It’s remarkable that regardless of the number of pixels selected in a frame, we only need four numbers to calculate their sum — simplicity in complexity!

By understanding how OpenCV and Harr Cascade models work and the algorithms behind them, we have successfully set up our Companion Robot to detect faces with OpenCV using a Raspberry PI. From detecting no faces (0), one face (1), to detecting multiple faces, we have shown you how it can be achieved effectively.

Remember, whether you're a seasoned developer or at the beginning of your tech journey, you can accomplish this task. Let’s continue building robots, coding, and having loads of fun!
