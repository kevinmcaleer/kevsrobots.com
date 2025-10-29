---
layout: project
title: Bluetooth remote controlled robot
description: "Make your own bluetooth remote controlled robot using a Raspberry Pi Pico W and MicroPython"
difficulty: Intermediate
short_title: Bluetooth controlled robot
short_description: "Control a robot with bluetooth"
date: 2023-07-03
author: Kevin McAleer
excerpt: >-
    Learn how to build your own Bluetooth remote controlled robot using a Raspberry Pi Pico and MicroPython
cover: /assets/img/blog/ble/ble.jpg
tags: 
 - Raspberry Pi Pico W
 - MicroPython
 - Bluetooth
 - Remote Control
groups:
 - pico
 - pcb
 - electronics
videos:
 - -0wCtKz1l78
repo:
 - https://gist.github.com/kevinmcaleer/cb8026c14ecb2b5a22fba065eb11af8c
---

## How To Use Bluetooth On Raspberry Pi Pico With MicroPython

Hello fellow Makers! Today, we're going to learn how to use Bluetooth on the Raspberry Pi Pico using MicroPython.

Back in mid-June this year, the Raspberry Pi team [announced that the Bluetooth functionality is now available for Raspberry Pi Pico](https://www.raspberrypi.com/news/new-functionality-bluetooth-for-pico-w/). Exciting, isn't it?

We'll upgrade our firmware, and create two programs; one for the remote control and one for the robot itself.

I've used the [BurgerBot](/burgerbot) robot as a platform for experimenting with bluetooth, and you can learn how to build your own using with the information in the link provided.

---

## Understanding Bluetooth Basics

Before we get started, let's dive into some Bluetooth basics. Bluetooth is a wireless communication technology used to exchange data over short distances. Invented by Ericsson in 1989, it was intended to replace RS-232 data cables to create wireless communication between devices.

Bluetooth operates between 2.4 and 2.485 GHz in the ISM Band, and typically has a range of up to a hundred meters. It's ideal for creating personal area networks for devices such as smartphones, PCs, peripherals, and even for controlling robots.

![BLE Slide](/assets/img/blog/ble/ble01.jpg){:class="img-fluid w-100"}

---

## Types of Bluetooth Technologies

There are two different types of Bluetooth technologies:

1. **Classic Bluetooth or Human Interface Devices (HID):** This is used for devices like keyboards, mice, and game controllers. It allows users to control the functionality of their device from another device over Bluetooth.

2. **Bluetooth Low Energy (BLE):** A newer, power-efficient version of Bluetooth, it's designed for short bursts of long-range radio connections, making it ideal for Internet of Things applications where power consumption needs to be kept to a minimum. 

![BLE Slide](/assets/img/blog/ble/ble02.jpg){:class="img-fluid w-100"}

---

## Step 1: Updating the Firmware

To access this new functionality, all we need to do is update the [firmware](https://github.com/pimoroni/pimoroni-pico/releases) on our Raspberry Pi Pico. This can be done either using an updater or by downloading the file from micropython.org and dragging it onto our Pico from the explorer or Finder window.

---

## Step 2: Establishing a Bluetooth Connection

A Bluetooth connection goes through a series of different stages. First, we need to advertise a service on the server (in our case, the Raspberry Pi Pico). Then, on the client side (the robot, for instance), we need to scan for any remote control nearby. Once it's found one, we can then establish a connection.

Remember, you can only have one connection at a time with Raspberry Pi Pico's implementation of Bluetooth in MicroPython. After the connection is established, we can transfer data (up, down, left, right commands to our robot). Once we're done, we can disconnect.

![BLE Slide](/assets/img/blog/ble/ble03.jpg){:class="img-fluid w-100"}

---

## Step 3: Implementing GATT (Generic Attribute Profiles)

GATT, or Generic Attribute Profiles, is used to establish the communication between two devices. However, it's only used once we've established the communication, not at the advertising and scanning stage.

To implement GATT, we will need to use asynchronous programming. In asynchronous programming, we don't know when a signal is going to be received from our server to move the robot forward, left, or right. Therefore, we need to use asynchronous code to handle that, to catch it as it comes in.

There are three essential commands in asynchronous programming:

1. `async`: Used to declare a function as a coroutine.

2. `await`: Used to pause the execution of the coroutine until the task is completed. 

3. `run`: Starts the event loop, which is necessary for asynchronous code to run.

![BLE Slide](/assets/img/blog/ble/ble04.jpg){:class="img-fluid w-100"}

![BLE Slide](/assets/img/blog/ble/ble05.jpg){:class="img-fluid w-100"}

---

## Step 4: Write Asynchronous Code

There is a module in Python and MicroPython that enables asynchronous programming, this is the `asyncio` (or `uasyncio` in MicroPython).

We can create special functions that can run in the background, with multiple tasks running concurrently. (**Note** they don't *actually* run concurrently, but they are switched between using a special loop when an `await` call is used). These functions are called `coroutines`.

Remember, the goal of asynchronous programming is to write non-blocking code. Operations that block things, like input/output, are ideally coded with `async` and `await` so we can handle them and have other tasks running elsewhere. 

The reason I/O (such as loading a file or waiting for a user input are blocking is because they wait for the thing to happen and prevent any other code from running during this waiting time).

It's also worth noting that you can have coroutines that have other coroutines inside them. Always remember to use the `await` keyword when calling a coroutine from another coroutine.

---

## The code

I've uploaded the working code to Github Gists so you can understand whats going on.

<script src="https://gist.github.com/kevinmcaleer/cb8026c14ecb2b5a22fba065eb11af8c.js"></script>

To use this code:

1. Upload the robot code to the robot and rename it to `main.py` - this will ensure it runs when the Pico is powered up.
2. Upload the remote code to the remote pico and rename it to `main.py`

The picos should flash quickly when not connected, and slowly once the connection is established.

---
