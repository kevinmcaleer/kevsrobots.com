---
layout: project
title: Googley Googley Eyes
description: "Googley Eyes, but they Google what they see"
difficulty: Intermediate
short_title: Googley Googley Eyes
short_description: "Googley Eyes, but they Google what they see"
date: 2023-07-26
author: Kevin McAleer
excerpt: Remember XEyes on the X Desktop, well its like that, but IRL
cover: /assets/img/blog/googleyeyes/googleyeyes.jpg
tags: 
 - Raspberry Pi
 - Python
 - opencv
 - Pimoroni
 - XEyes
groups:
 - weird
 - raspberrypi
 - electronics
videos:
 - bV2sw7XBaqo
repo:
 - https://gist.github.com/kevinmcaleer/8bf03bf74ac6cbf43314c41582d1e471
---

You’ve seen googley eyes or xeyes, but what if the googley eyes could Google what they see (and then announce it); I present googley googley eyes. (Ok, they don't actually 'Google' the image, they actually use something far more cool - OpenAI).

The eyes rotate using the motors with encoders for accurate positioning. This means they can rotate all the way round, but also rotate to a specific position, which is needed for making the eyes look in a direction accurately.

![Googley Eyes on workbench](/assets/img/blog/googleyeyes/googleyeyes01.jpg){:class="img-fluid w-75 rounded-3 shadow-lg"}

---

## Bill of Materials

Item             | Description                                       | Qty |   Cost
-----------------|---------------------------------------------------|:---:|------:
Pi Zero 2W       | Raspberry Pi Zero 2W                              |  1  | £17.10
Pi Camera        | Raspberry pi camera                               |  1  | £36.90
Camera Cable     | Camera Cable - Pi Zero Edition 150mm              |  1  |  £3.90
Motor Controller | Pimoroni Inventor HAT Mini                        |  1  | £24.00
Motors           | 2x MMME motors with encoders                      |  2  |  £9.90
Motor Cable      | Pimoroni JST-SH cable - 6 pin (pack of 4) - 300mm |  1  |  £3.90
M2 Screws        | Screws for the camera mount                       |  2  |  £0.50
M2.5 Screws      | Screws for the Raspberry Pi Zero 2W               |  4  |  £0.50
Speaker          | Mini Oval Speaker - 8 Ohm 1 Watt                  |  1  |  £2.70
{:class="table table-striped"}

---

## 3d printed parts

Here are the 3d printed parts for the project:

* [`pi_holder.stl`](/assets/stl/googley_eyes/pi_holder.stl) - Holds the Raspberry Pi, this is from a modular robotics system I'm working on
* [`eye_holder`](/assets/stl/googley_eyes/eye_holder.stl) - Holds the motors and provides a desktop mount for the project
* [`eye.stl`](/assets/stl/googley_eyes/eye.stl) - These are the white eye pieces, you'll need to print two of them
* [`pupil.stl`](/assets/stl/googley_eyes/pupil.stl) - These are the black, pupil pieces, you'll need to print two of them

![Googley Eyes on workbench](/assets/img/blog/googleyeyes/googleyeyes02.jpg){:class="img-fluid w-75 rounded-3 shadow-lg"}

---

## Assembley

1. Setup the Raspberry Pi with a fresh install of Raspberry Pi OS 64-bit, [Click here for a how to video](https://youtu.be/xCCzzYBWHrM)

1. Push the Pimoroni Inventor HAT Mini onto the top of the Raspberry Pi Zero 2W

1. Connect the motors up to the Inventor Hat Mini

1. Push the motors into the `eye_holder`

1. If you have access to a vinyl cutting machine such as a Cricut Maker 3, you can cut out a circle 50mm in diameter for the white eye, and 30mm for the black pupil. The White eye has a small 7mm hole in the center. The 3d printed parts may require some filing to ensure a smooth motion

1. Push the black `pupil` 3d printed part into the white eye piece

1. Glue the `eye` 3d printed disk onto the `eye_holder` with some super-glue, be sure not to glue the pupil part ensuring it can still turn friction free

1. Screw the `Raspberry Pi Zero 2W` into the `pi_holder` using the M2.5 screws

1. Carefully push the `Raspberry Pi camera cable` into both the `Raspberry Pi camera module` and the `Raspberry Pi Zero 2W`

1. Screw the `Raspberry Pi camera module` into the `eye_holder` with some M2 screws

1. Push the small PicoBlade speak cable into the speaker connector on the Inventor HAT Mini

![Googley Eyes on workbench](/assets/img/blog/googleyeyes/googleyeyes03.jpg){:class="img-fluid w-75 rounded-3 shadow-lg"}

---

## Setting up the image recognition

OpenAI has a nice automatic image captioning service, you can get this up and running by:

1. Download the project code:

    ``` bash
    git clone https://www.github.com/kevinmcaleer/googley_eyes
    ```

1. Sign up for an account on <https://platform.openai.com>

1. Create a new API key: Click on your profile picture at the top right, and then `View API Keys`, then click `create new secret key`

1. Copy the secret API key into a file named `.env` into the downloaded code:

    ``` python
    OPENAI_API_KEY=<INSERT YOUR OPENAI API KEY HERE>
    ```

1. Create a virtual environment:

    ``` bash
    python3 -m venv venv
    ```

1. Activate the environment:

    ``` bash
    source venv/bin/activate
    ```

1. Install the dependencies:

    ``` bash
    pip install -r requirements.txt
    ```

1. Running the demo program `caption_this.py`:

    ``` bash
    python3 caption_this.py
    ```

1. You can upload your own image and update the code that loads the image.

## Tweak the code

You can also tweak the query, to ask OpenAI how to describe the image, here is a response generated from the query:

``` python
result = index.query('describe what is in the image, be nonchalant and snarky')
```

> ### Results
>
> ![Archie and Trixie](/assets/img/blog/googleyeyes/archie_and_trixie.jpg){:class="img-fluid w-75"}
>
> ***"A cat and dog lounging on a blanket, like they own the place."***

---

## Making the eyes move

The eyes can be moved using the Pimoroni Inventor HAT mini python library; for instructions on how to install that, [click here](https://github.com/pimoroni/inventorhatmini-python).

Once you've installed that, headover to the motors example code and run the motor_wave.py program:

``` bash
cd InventorHATMini/examples/motors
python3 motor_wave.py

```

---

## Code

Here is the code you will need, please note this is a work in progress so the code will improve over time.

<script src="https://gist.github.com/kevinmcaleer/8bf03bf74ac6cbf43314c41582d1e471.js"></script>

---
