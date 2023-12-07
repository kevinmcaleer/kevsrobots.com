---
title: Elf detector
description: A Raspberry Pi powered robot that can find elves!
layout: project
date: 2023-12-05
cover: /assets/img/blog/elf/elf.jpg
excerpt:
author: Kevin McAleer
difficulty: Intermediate
groups:
    - robot
    - micropython
    - christmas
    - raspberrypi
tags:
    - robots
    - ai
    - machine learning
    - opencv
    - viam
---

[`Viam`](https://app.viam.com) has a really easy to use machine learning training system that can be used to train a model to recognise objects. I thought it would be fun to train a model to recognise Elves, and then use it on a robot that could find them.

---

## The robot

I'm using an off the shelf `Rover` robot from Viam, which is powered by the Raspberry Pi 4. The robot has a webcam connected and this can be used to capture images our our elf from different positions to train our model. It then use the camera later to detect elves within the robots field of view.

---

## Creating a robot in Viam

To create a robot in Viam, you need to log into the Viam software at [app.viam.com](https://app.viam.com). Once you have logged in, you can create a new robot by adding the name of the new robot and clicking on the `Add Robot` button in the top left corner of the screen.

![Viam create robot](/assets/img/blog/elf/add_robot.png){:class="img-fluid w-100 shadow-lg"}

---

## Installing Viam on the Raspberry Pi

To install the `viam-server` on the Raspberry Pi, follow 3 the onscreen instructions below. This assumes you've some knowledge about how to setup and use an SSH session to connect to your Raspberry Pi. Luckily, Viam has a really easy to follow guide to help you do this, available [here](https://docs.viam.com/get-started/installation/prepare/rpi-setup/).

1. **Download the Viam app** - copy and paste the command into an ssh session connected to your raspberry pi

1. **Download and install the viam-server service** - again copy and paste the command into an ssh session connected to your raspberry pi.

1. **Check for connectivity** - Finally, Viam will report back that it has detected that the viam-server is now running on the Raspberry Pi and will present a button to configure the robot.

![Installing Viam Screenshot](/assets/img/blog/elf/install.png){:class="img-fluid w-100 shadow-lg"}

---

## Configuring the robot

Next we need to add a camera to our robot so that it can see. The easiest way to do this on Viam and Raspberry Pi is to plug in a cheap USB Webcam.

To add this to our robot in Viam, we need to:

1. Click on the `Config` tab on the robot we want to add the camera to. This will open up the configuration screen for the robot
1. Click the `+ Create Component` button to add a new component to the robot
1. Select `Camera` from the list of components
1. Select `Webcam` from the list of cameras
1. Give the camera a name
1. Choose `video0` for the Video Path
1. Click `Create Component`

---

## Training the model

The Viam software has a really easy to use training system. You can train a model to recognise objects in just a few minutes. I used the webcam on the robot to capture images of our elf from different angles, and then used the Viam software to train the model.

To do this:

1. log into [app.viam.com](https://app.viam.com) and select the robot you want to train from the list of robots on the left hand side of the screen.

