---
layout: blog
title: Simple Robot Arm
short_title: Robot Arm
short_description: Build your own Robot Arm
description: Build your own Robot Arm
date: 2022-09-25
author: Kevin McAleer
excerpt: 3D print a simple robot arm you can control over Wi-Fi using Phew! and the Pimoroni Inventor 2040 W
cover: /assets/img/blog/robotarm/robotarm.jpg
tags:
 - Raspberry Pi Pico W
 - Inventor 2040 W
 - Pico
 - Pico W
 - Robot
 - MicroPython
 - Logo
 - Arm
 - 3D Printing
groups:
 - robots
 - 3dprinting
 - pico
 - micropython

---

## Table of Contents

{:toc}
* toc

---

## Video
Here is a link to the YouTube video that covers the operation and programming of the Simple Robot Arm.

{% include youtubeplayer.html id="HmZF6e3I1Lo" %}

---

## Grippy-Bot
Grab the 3d printable STL files for the Grippy-Bot from Cult 3d - <https://cults3d.com/en/3d-model/gadget/grippy-bot>. This model is a free to download, and is pretty quick to 3d print. I used the Cura slicer with 'Standard' print settings and these worked out fine. I managed to print all the items on the build plate at the same time.

[![Picture of the completed 3d printed robot arm](/assets/img/blog/robotarm/robotarm.jpg){:class="img-fluid w-100 rounded"}](/assets/img/blog/robotarm/robotarm.jpg)

---

## Bill of Materials

Part            | Description                                                                                                                         | Qty | Cost
----------------|-------------------------------------------------------------------------------------------------------------------------------------|-----|-------
SG90 Servo      | These are the motors that move the parts of the arm                                                                                 | 5   | £4.00
Inventor 2040 W | This is the brains of the Robot arm and will provide Wi-Fi access. Available from [Pimoroni](https://www.pimoroni.com/inventor2040w) | 1   | £34.50
{:class="table table-striped"}

You'll probably find packs of Servos online cheaper than buying them individually.

---

## MicroPython Code

This project enables you to control the robot arm over Wi-Fi, using the amazing [Phew!](https://www.github.com/pimoroni/phew) library from Pimoroni. Phew provides simple functions for creating an Access Point, serving Webpages from the Pico W and a templating engine for embedding variables in those web pages.

Download the code from here: <https://www.github.com/kevinmcaleer/inventor_arm>

You'll need to install Phew! on the Pico W, and copy over the `index.html` file too. You can upload files using Thonny (Right click on the files and folders to upload from the thonny files menu). If you want to know more about how to install Phew! [Watch this part of this video](https://youtu.be/0sPPxIq4hg8?t=241)

There are two MicroPython programs in the repository:
- `test01.py` - A simple test program to check the motors are working - they will all move from their minimum to maximum values
- `test02.py` - This is the main program, it will display a web-based user interface on the IP address logged to the console. Just type that address into the web browser of any device, phone, table or computer on the same network to access the Robot Arm webpage.

---

### Overview of the code

There are two parts to the robot arm code - the MicroPython that runs on the Inventor 2040 W, and the Javascript that runs on the browser, from the `index.html` file.

### MicroPython code

The Wi-Fi SSID and Password are defined in a `config.py` file on the Pico W - if this doesn't exist you'll need to create it with the following content:

``` python
wifi_ssid = '<enter your Wi-Fi name here>'
wifi_password = '<enter your Wi-Fi password here>'
```

The first block of code brings in all the software libraries needed for this project, including the phew! library, and the Inventor 2040 W library. 


``` python
from phew import *
from phew import connect_to_wifi, server, logging
from phew import render_template
from config import wifi_ssid, wifi_password
from inventor import Inventor2040W, SERVO_1, SERVO_2, SERVO_3, SERVO_4, SERVO_5, SERVO_6 
from time import sleep
import math

```

The next block of code connects to the Wi-Fi network, creates a `board` variable and sets each of the servos to its middle position.

``` python
# Connect to Wi-Fi
ip = connect_to_wifi(wifi_ssid, wifi_password)

# Create a new board
board = Inventor2040W()

# Set all servos to mid position
for servo in board.servos:
    servo.to_mid()
    print(f'servo: {servo}, value {servo.value()}')
    sleep(0.1)

```

We create a `position` function to take values passed to it from the webserver and sets the servos accordingly.

``` python
def position(arm=None, wrist=None, elbow=None, finger=None, base=None):
    """ Set the servo positions """
    if finger is not None:
        board.servos[SERVO_2].value(finger)
    if wrist is not None:
        board.servos[SERVO_3].value(wrist)
    if arm is not None:
        board.servos[SERVO_4].value(arm)
    if elbow is not None:
        board.servos[SERVO_5].value(elbow)
    if base is not None:
        board.servos[SERVO_6].value(base)
    sleep(0.01)

```

The next block of code listens to any web requests to the `/` url and either send the rendered `index.html` file back to the users browser, or processes the servo slider positions, one for each of the servos.

``` python
@server.route('/', methods=['GET','POST'])
def index(request):

    if request.method == 'GET':
        return render_template('index.html')
    elif request.method == 'POST':
        
        elbow = request.form.get("elbow", None)
        arm = request.form.get("arm", None)
        base = request.form.get("base", None)
        finger = request.form.get("finger", None)
        wrist = request.form.get("wrist", None)
        if elbow is not None:
            position(elbow=int(elbow))
        if arm is not None:
            position(arm=int(arm))
        if base is not None:
            position(base=int(base))
        if wrist is not None:
            position(wrist=int(wrist))
        if finger is not None:
            position(finger=int(finger))

        # Try without the line below to speed up the response
        return render_template('index.html')
```

The last block of code shows the IP address of the Inventor 2040 W after it connected to Wi-Fi earlier, and then starts the Webserver.

```python
# Show the IP Address
logging.info(f'IP: {ip}')
logging.is_disabled = True

# Start the server
server.run()
```

---

### Javascript

The other code we need runs on the users browser and is therefore written in Javascript:

``` javascript
function post_values()
{
  $.post(
  "",
  { elbow: elbow.value, base: base.value, arm: arm.value, wrist: wrist.value, finger: finger.value },
  function(data) {
    
  }
);
};
```

This code takes the values from each of the sliders and posts them to the webserver. It uses **JQuery** to simplify the code required to post values back to the webserver.  Javascript can look a bit esoteric to the uninitiated - I'm not a massive fan of the language as its now obvious whats going on from the code itself, and it requires a firm understanding of the grammar and syntax to have the vaguest of clues as to what is going on.

---


## Wiring up the Servos to the Inventor 2040 W

Wiring up the robot arm is pretty simple - just plug each of the servo connectors into the Inventor 2040 W using the connections as shown below:

Servo  | Inventor 2040
-------|--------------
Finger | Servo 2
Wrist  | Servo 3
Arm    | Servo 4
Elbow  | Servo 5
Base   | Servo 6
{:class="table table-sm"}


[![Picture of the Wiring diagram](/assets/img/blog/robotarm/wiring.jpg){:class="img-fluid w-50 shadow-lg"}](/assets/img/blog/robotarm/wiring.jpg)

---

## The Web UI

The Web UI has 5 sliders - one for each of the servos. The middle position corresponds to the Servos middle position.

[![Picture of the Web UI](/assets/img/blog/robotarm/ui.jpg){:class="img-fluid w-50 shadow-lg"}](/assets/img/blog/robotarm/ui.jpg)

If you move the slider and then release the mouse button, the servo will then move to the position selected. The Pico Phew! library will find it difficult to move the servo in realtime sync with the slider - ask me how I know!

---

## Conclusion

This is a great starter project to experiment with robotics. There are lots of area to improve upon this project including:

* stronger servos
* servo position feedback (requires a higher quality servo with feedback wiring)
* more robot 3d design
* longer arm for further reach
* double jointed servo claw to enable better and more precise handling of objects with the claw