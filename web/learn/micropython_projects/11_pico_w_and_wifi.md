---
layout: lesson
title: Pico W and Wi-Fi
author: Kevin McAleer
type: page
cover: /learn/micropython_projects/assets/banners/11_pico_w_and_wifi.jpg
date: 2026-11-24
previous: 10_giving_the_buddy_a_voice.html
next: 12_arduino_cloud.html
description: Meet the Pico W, understand what happens when it joins a network, and
  connect it in a few lines of MicroPython
percent: 60
duration: 6
quiz_total: 19
date_updated: 2026-11-24
navigation:
- name: MicroPython Projects - Make It Show, Talk and Connect
- content:
  - section: Getting started
    content:
    - name: Introduction
      link: 00_intro.html
    - name: Meet the status buddy
      link: 01_meet_the_status_buddy.html
    - name: Parts and pin plan
      link: 02_parts_and_pin_plan.html
  - section: Sense
    content:
    - name: Reed switches and magnets
      link: 03_reed_switches_and_magnets.html
    - name: Reading the door
      link: 04_reading_the_door.html
    - name: Interrupts and debouncing
      link: 05_interrupts_and_debounce.html
  - section: Show
    content:
    - name: OLED wiring and first text
      link: 06_oled_wiring_and_first_text.html
    - name: Drawing shapes and icons
      link: 07_drawing_shapes_and_icons.html
    - name: The status screen
      link: 08_the_status_screen.html
  - section: Talk
    content:
    - name: Meet SAM
      link: 09_meet_sam.html
    - name: Giving the buddy a voice
      link: 10_giving_the_buddy_a_voice.html
  - section: Connect
    content:
    - name: Pico W and Wi-Fi
      link: 11_pico_w_and_wifi.html
    - name: Arduino Cloud
      link: 12_arduino_cloud.html
  - section: Shrink it
    content:
    - name: Porting to a XIAO
      link: 13_porting_to_a_xiao.html
  - section: Putting it together
    content:
    - name: The full gadget
      link: 14_the_full_gadget.html
    - name: 15_troubleshooting.md
      link: 15_troubleshooting.html
    - name: Summary
      link: 16_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

The buddy is great, until you walk into the next room and can't see the screen or hear the speaker. Time to go wireless. This begins the **Connect** section.

Kev's video covers Wi-Fi on the Pico W, local web servers and Arduino Cloud:

{% include youtubeplayer.html id="Wwhmy9AqCbE" %}

---

## Begin with the end in mind

Before any code, ask the question Kev asks at the start of his Wi-Fi episode: **what do you want the data to do?** Sit on a web page? Buzz your phone? Plot a chart? Each answer leads to different code.

For the buddy, the answer is: *tell my phone whether the door is open, from anywhere.* That points to a cloud service, which is the next lesson. First we need to get the Pico onto Wi-Fi.

## Meet the Pico W

A Pico W has the same pins, the same code and the same MicroPython as a regular Pico, plus one extra chip, the **CYW43439**. It gives you:

- **2.4 GHz** Wi-Fi (not 5 GHz, which matters, see below)
- **Bluetooth 5.2**

Everything we have wired so far, the reed switch, OLED and speaker, works exactly as it did. We are just adding a radio.

## The post office of the internet

When a Pico joins a network, four helpers do their jobs. Kev explains them as a little town:

| Helper | Role in the town | What it gives your Pico |
|---|---|---|
| DHCP | The postman | A house number: your IP address, such as `192.168.1.50` |
| MAC address | The name on the deed | A permanent ID burned into the Wi-Fi chip |
| Default gateway | The front gate | The way out to the wider internet. Usually your router |
| DNS | The phone book | Turns names like `arduino.cc` into numbers |
{:class="table table-single"}

One call to `wlan.connect()` sets up all four.

## Keep secrets in secrets.py

Never type your Wi-Fi name and password into a program you might share. Make a small file called `secrets.py` and save it on the Pico:

```python
# secrets.py - lives on the Pico only. Never put this file on GitHub
WIFI_SSID = "YOUR_WIFI_NAME"
WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"
```

Those two values are **placeholders**. Replace them with your own, and keep the file out of screenshots and screen recordings. In lesson 12 we add the Arduino Cloud keys to the same file.

## Wi-Fi in a few lines

```python
import network
import time
from secrets import WIFI_SSID, WIFI_PASSWORD

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(WIFI_SSID, WIFI_PASSWORD)

while not wlan.isconnected():
    print("Connecting...")
    time.sleep(1)

print(wlan.ifconfig())
```

What each part does:

- `STA_IF` is "station mode": your Pico is a *client* on someone else's network
- `active(True)` switches the radio on
- `connect(...)` asks for a house number, finds the gate and sets up DNS
- `isconnected()` stays false until the router has agreed
- `ifconfig()` prints the IP address, subnet mask, gateway and DNS server

You should see something like `('192.168.1.50', '255.255.255.0', '192.168.1.1', '192.168.1.1')`. There is your whole town.

## The buddy gets online

Let's let the buddy tell us when it's connected. This program gives up after 20 seconds instead of hanging forever, shows the result on the OLED, and says it out loud.

```python
import network
import time
from machine import Pin, I2C
from ssd1306 import SSD1306_I2C
from sam import Sam
from secrets import WIFI_SSID, WIFI_PASSWORD

i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=400000)
oled = SSD1306_I2C(128, 64, i2c)
sam = Sam(pin=0)


def show(line1, line2=""):
    oled.fill(0)
    oled.text("Status Buddy", 16, 0)
    oled.hline(0, 10, 128, 1)
    oled.text(line1, 0, 24)
    oled.text(line2, 0, 40)
    oled.show()


show("Connecting...")

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(WIFI_SSID, WIFI_PASSWORD)

start = time.ticks_ms()
while not wlan.isconnected():
    if time.ticks_diff(time.ticks_ms(), start) > 20000:
        break
    time.sleep(1)

if wlan.isconnected():
    ip = wlan.ifconfig()[0]
    show("Wi-Fi connected", ip)
    sam.say("Wi-Fi connected")
else:
    show("No Wi-Fi", "Check secrets.py")
    sam.say("No Wi-Fi")
```

## Wi-Fi is the road, not the vehicle

Being on Wi-Fi doesn't send anything anywhere. It is only the road. To send data, you pick a vehicle:

| Vehicle | Like a... | Good for |
|---|---|---|
| HTTP | Postal service | Fetching or serving web pages |
| MQTT | Radio station | Many sensors broadcasting to many listeners |
| Cloud SDK | Phone line kept open | A service that stays in touch, like Arduino Cloud |
{:class="table table-single"}

And where the data **lives** matters. If the Pico serves a web page, your phone needs to be on the same Wi-Fi. If the Pico pushes to a cloud service, your phone can be on the moon. We want the moon.

## The three gotchas

1. **2.4 GHz only.** The Pico W can't see 5 GHz networks. If your router shares one name between both bands, give each band its own name.
2. **Everything is case-sensitive.** `Home_WiFi` and `home_wifi` are different networks, and the password is case-sensitive too. Copy them from your router rather than retyping.
3. **`[Errno 98] Address already in use`.** This is a web server problem. If you write a Pico web server and restart it, the port stays locked for about a minute. `s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)` solves it. We don't need a web server here, but you'll meet this one if you build one.

## Try it Yourself

1. Print `wlan.ifconfig()[0]` and write down your Pico's IP address.
2. Unplug the Pico, move the router away (or change the password in `secrets.py` on purpose) and watch the "No Wi-Fi" message appear.
3. Challenge: Kev's Wi-Fi post builds a tiny web server with the `socket` module. Read it and think about how you might serve the door state as a page.

## Common Issues

**Problem**: It connects on my phone's hotspot but not on my home Wi-Fi.

**Solution**: Check your router is broadcasting 2.4 GHz and that the name has no 5 GHz twin with the same name.

**Why**: The Pico W's radio only speaks 2.4 GHz.

**Problem**: `ImportError: no module named 'secrets'`.

**Solution**: Save `secrets.py` on the Pico itself, in its root folder.

**Why**: Imports come from the Pico's filesystem, not your computer.

**Problem**: The Wi-Fi code gives an error on my plain Pico.

**Solution**: Wi-Fi needs a Pico W, running MicroPython built for the W. Everything else in the course works on a plain Pico, so skip the Connect lessons and leave `USE_CLOUD` set to `False` in lesson 14.

**Why**: A plain Pico has no Wi-Fi chip.
