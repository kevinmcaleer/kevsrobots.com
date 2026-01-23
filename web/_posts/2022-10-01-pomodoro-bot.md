---
layout: project
title: Pomodoro Robot! 
short_title: Pomodoro Desk Robot
short_description: Build your own Pomodoro Desk bot
description: Build your own Pomodoro Desk bot
difficulty: Beginner
date: 2022-10-02
author: Kevin McAleer
excerpt: Build your own cute Pomodoro Desk Robot - HeyBot! Using a Raspberry Pi Pico W and Pimoroni Display Pack 2.0
cover: /assets/img/blog/heybot/heybot.jpg
tags:
 - pico_w
 - pico
 - robot
 - micropython
 - pomodoro
 - 3d_printing
groups:
 - robots
 - 3dprinting
 - pico
 - micropython 
 - pets
videos:
 - /MWg1xdmgE04
 - /D2SgH5qxJuI
repo:
 - https://www.github.com/kevinmcaleer/heybot
---

## HeyBot! The Pomodoro timer Desk Robot

HeyBot! is a Pomodoro timer desk robot you can use to increase your productivity.

---

## Bill of Materials

Part                                                                         | Description                                            | Qty | Cost
-----------------------------------------------------------------------------|--------------------------------------------------------|-----|-------
Raspberry Pi Pico W                                                          | The $6 microcontroller from Raspberry Pi               | 1   | £6.00
[Display Pack 2.0](https://shop.pimoroni.com/products/pico-display-pack-2-0) | Pimoroni 2" Display that plugs into the Pico W Headers | 1   | £18.90
M2 Bolts | Securely attach the Pico W to the Head using M2 Bolts | 4 | £0.10
{:class="table table-striped"}

> Prices and availability may vary.

---

## MicroPython Code

Download or clone the code here: <https://www.github.com/kevinmcaleer/heybot>

The code for HeyBot consists to two main parts:

* `countdowntimer.py` - Countdown Timer class
* `pomodoro.py` - the main program

This project also uses a couple of Pimoroni MicroPython libraries:
* [PicoGraphics](https://github.com/pimoroni/pimoroni-pico/tree/main/micropython/modules/picographics) - graphics library 
* [Phew!](https://github.com/pimoroni/phew) - the Pico HTTP Endpoint Wrangler, for quick connection to Wi-Fi, and NTP time library
* [Pimoroni-Pico](https://github.com/pimoroni/pimoroni-pico/releases) - the 'batteries included' build of MicroPython

> **NOTE**:
>
> Be sure to copy all the jpeg images to the Pico W; Thonny makes this easy, just select them in the file browser, right click and upload to the Pico.
{:class="blockquote  note m-3 p-3 rounded-1"}

---

### CountDownTimer Class

``` python
# Countdown timer

from machine import RTC
import time

class CountDownTimer():
    hours = 0
    minutes = 0
    seconds = 0
    duration = 25 # minutes default
    duration_in_seconds = duration * 60
    alarm = False
    
    def __init__(self):
        self.start_time = time.time()
        print(f'start time is :{self.start_time}')
    
    @property
    def duration(self):
        """ Return the duration in minutes """
        return self.duration_in_seconds * 60
    
    @duration.setter
    def duration(self, duration_in_minutes):
        """ Set the duration in minutes """
        self.duration_in_seconds = duration_in_minutes * 60
            
    @duration.setter
    def duration_seconds(self, duration):
        """ Set the duration in seconds """
        self.target_time = time.time() + duration
        self.duration_in_seconds = duration
            
    def reset(self):
        """ Reset the timer, and turn off the alarm """
        self.start_time = time.time()
        self.alarm = False
    
    def time_to_str(self,time_as_number)->str:
        """ return the current time as a pretty string of text """
        target = time.localtime(time_as_number)
        hours =  target[3]
        minutes = target[4]
        seconds =  target[5]
        return f'{hours:02}:{minutes:02}:{seconds:02}'
    
    @property
    def start_time_str(self)->str:
        """ Return the start time as a pretty string of text """
        target = self.start_time

        return self.time_to_str(target)
    
    @property
    def target_time(self)->int:
        """  Return the target time as an integer """
        # add duration in seconds to epoc
        target = self.start_time + (self.duration_in_seconds)
        return target
    
    @property
    def target_str(self)->str:
        """ Return the target time as a pretty string of text """

        return self.time_to_str(self.target_time)
    
    @property
    def remaining_str(self)->str:
        """ Return the remaining time as a pretty string of text """
        time_left = self.remaining_seconds       

        return self.time_to_str(time_left)
    
    @property
    def current_time_str(self)->str:
        """ Return the current time as a pretty string of text """
        current = time.time()

        return self.time_to_str(current)
    
    @property
    def remaining_seconds(self)->int:
        """ returns remaining seconds as an integer """
        
        remaining = self.target_time - time.time()
        if remaining > 0:
            
            return remaining
        else: return 0
    
    def isalarm(self)->bool:
        """ Returns the state of the Alarm, as a boolean - True of Ralse"""
        if self.remaining_seconds == 0:
            self.alarm = True
            return True
        else:
            return False
    
    def tick(self):
        """ Return the remaining time as a pretty string of text """
        # Get time as tuple
        # (year, month, mday, hour, minute, second, weekday, yearday)
       
        return self.remaining_str
    
    def status(self):
        """ Print the current status, useful for debugging """
        if not self.isalarm():
            print(f'Start time    : {self.start_time_str}', end='')
            print(f' | Current time  : {self.current_time_str}', end='')
            print(f' | Target time   : {self.target_str}', end='')
            print(f' | Remaining time: {self.remaining_str}')
```

---

### Main Pomodoro Program

The code below is the main program, it shows the current time, countdown timer and an animated face.

``` python
# pomodoro

from phew import connect_to_wifi, logging
from phew.ntp import fetch
from config import wifi_ssid, wifi_password
import usocket
import jpegdec
import struct
from time import sleep, gmtime, time
from machine import RTC
from picographics import PicoGraphics, DISPLAY_PICO_DISPLAY_2
from random import choice
from countdowntimer import CountDownTimer
from pimoroni import Button

display = PicoGraphics(display=DISPLAY_PICO_DISPLAY_2, rotate=180)

# Get the screen dimensions
WIDTH, HEIGHT = display.get_bounds()

EYES = 'eyes.jpg'

# set up buttons
button_a = Button(12)
button_b = Button(13)
button_x = Button(14)
button_y = Button(15)

# Setup the animation frames for Angry face
angry_frames = ['angry01.jpg',
         'angry02.jpg',
         'angry03.jpg',
         'angry04.jpg',
         'angry05.jpg',
         'angry06.jpg',
         'angry07.jpg']

# Setup the animation frames for Normal face
normal_frames = ['normal01.jpg',
                 'normal02.jpg',
                 'normal03.jpg',
                 'normal04.jpg']

# Setup the animation frames for Static face
static_frames = ['eyes.jpg',
                 'eyes.jpg']

# Define the pen colors - used for drawing text and shapes
RED = display.create_pen(255,0,0)
WHITE = display.create_pen(255,255,255)
BLACK = display.create_pen(0,0,0)
    
class Animate():
    """ Models animations """
    direction = 'forward'       # Set the direction of the animation forward or backward
    frame = 1                   # Current frame   
    frames = []                 # list of all frames
    is_done_animating = False
    
    def animate(self, display):
        """ Animate the frames """
        if self.direction == 'forward':
            self.frame += 1
            
            if self.frame > len(self.frames):
                self.direction = 'backward'
                self.frame = len(self.frames)
        else:
            self.frame -= 1
            if self.frame < 1:
                self.direction = 'forward'
                self.frame = 1
                self.is_done_animating = True
        
        # Draw the current frame
        draw_jpg(display,self.frames[self.frame-1])

def draw_jpg(display, filename):
    """ Display a JPEG on the display, best if the image is the same size as the display """
    j = jpegdec.JPEG(display)

    # Open the JPEG file
    j.open_file(filename)

    # Get the screen dimensions and clip image if necessary
    WIDTH, HEIGHT = display.get_bounds()
    display.set_clip(0, 0, WIDTH, HEIGHT)

    # Decode the JPEG
    j.decode(0, 0, jpegdec.JPEG_SCALE_FULL)
    display.remove_clip()

def update_clock(max_attempts = 5):
    """ Update the clock from the internet """
    ntp_host = 'pool.ntp.org'
    attempt = 1
    while attempt < max_attempts:
        try:
            query = bytearray(48)
            query[0] = 0x1b
            address = usocket.getaddrinfo(ntp_host, 123)[0][-1]
            socket = usocket.socket(usocket.AF_INET, usocket.SOCK_DGRAM)
            socket.settimeout(30)
            socket.sendto(query, address)
            data = socket.recv(48)
            socket.close()
            local_epoch = 2208988800
            timestamp = struct.unpack("!I", data[40:44])[0] - local_epoch
            t = gmtime(timestamp)
            if not t:
                logging.error(" - failed to fetch time from ntp server")
                return False
            RTC().datetime((t[0], t[1], t[2], t[6],t[3],t[4],t[5],0))
            logging.info(" - rtc synced")
            return True
        except Exception as e:
            logging.error(e)
            
        attempt += 1
    return False

def banner(display, bg_colour, fg_colour):
    """ Display a coloured banner on the display """
    display.set_pen(bg_colour)
    display.rectangle(0,210,WIDTH,HEIGHT)
    display.set_pen(fg_colour)

# ------------------ Main Program ------------------

# connect to Wi-Fi
draw_jpg(display,EYES)
logging.debug('about to connect to Wi-Fi')
connect_to_wifi(wifi_ssid, wifi_password)

# update the clock
t = update_clock()

# Create a countdown timer
countdown = CountDownTimer()

# Set the countdown timer to 25 minutes
countdown.duration = 25

# Set the font
display.set_font("bitmap8")

# log the current time
current_time = countdown.current_time_str
print(current_time)

# Set the default drawing coordinates
x = 1
y = 1
scale = 4
angle = 0
spacing = 1
wordwrap = False
display.set_pen(15)

# Setup Animations
animations = [angry_frames,normal_frames, static_frames]
animation = Animate()
animation.frames = choice(animations)

# Start the timer
countdown.reset()

# The main loop
while True:
    
    # Read button states
    if button_y.read():
        print("button Y")
        countdown.reset()
    if button_a.read():
        print("button A")
        countdown.reset()
    
    # Update the time display 
    current_time = countdown.current_time_str
    display.set_pen(0)
    display.clear()

    remaining_time = countdown.remaining_str
    
    # Animate the face
    if not animation.is_done_animating:
        animation.animate(display)
    else:
        animation.frames = choice(animations)
        animation.is_done_animating = False
        animation.animate(display)
    
    # Display the countdown timer
    display.set_pen(15)
    x = WIDTH // 2 - (display.measure_text(current_time, scale, spacing) //2 )
    display.text(current_time, x, y, wordwrap, scale, angle, spacing)
    
    x = WIDTH // 2 - (display.measure_text(remaining_time, scale, spacing) //2 )
    if countdown.alarm:
        print('countdown done')
        if gmtime()[5] % 2 == 0 :
            banner(display,RED, WHITE)
        else:
            banner(display,BLACK,RED)
    else:
        display.set_pen(RED)
    display.text(remaining_time, x, y+210, wordwrap, scale, angle, spacing)
    
    # Update the display
    display.update()
```
---

## Wiring up the Robot - plug and play

Screw the Pico W into the Head using the mount points, using 4x M2 Bolts.

[![Pico W Headers](/assets/img/blog/heybot/pico.jpg){:class="img-fluid w-100"}](/assets/img/blog/heybot/pico.jpg)

The Pico W simply pushed onto the Pico Display Pack 2.0. Ensure you align the USB graphic on the back of the Pico Display Pack to make sure its the correct way round.


---

## The STL files

There are 4 parts to download and print:

* [`head.stl`](/assets/stl/heybot/head.stl) - the robot head
* [`body.stl`](/assets/stl/heybot/body.stl) - the robot body
* [`left_arm.stl`](/assets/stl/heybot/left_arm.stl) - the left arm
* [`right_arm.stl`](/assets/stl/heybot/right_arm.stl) - the right arm.

If you enjoy these files, please consider [buying me a coffee](/coffee) (it took a while to design these!)