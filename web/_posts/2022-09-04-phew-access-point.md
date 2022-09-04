---
layout: blog
title: How to setup a Phew! Access Point
short_title: Phew! Access Point
short_description: MicroPython Captive Portal
date: 2022-09-04
author: Kevin McAleer
excerpt: Setup your own Wifi Captive Portal for Robotics project
cover: /assets/img/blog/phew/saddle.jpg
tags:
 - Raspberry Pi
 - WiFi
 - Captive Portal
 - Pimoroni
 - Phew!
 - MicroPython
 
---

## Table of Contents

{:toc}
* toc

---

## Phew! That was too easy

You can quickly setup an Access Point / Captive Portal using MicroPython, a Raspberry Pi Pico W and the [Phew!](https://github.com/pimoroni/phew) library from [Pimoroni](https://www.pimoroni.com).

In fact its just a single line to setup the access point:

> ``` python
> ap = access_point("My Wifi Access Point")
> ```

![Access Point on an iPhone screen](/assets/img/blog/phew/access_point_iphone.jpg){:class="img-fluid w-25"}

Picture of an iPhone showing the 'My WiFi Access Point'
{:class=" small text-mute"}

---

## About Phew!

Phew! is a Pico Http Endpoint Wrangler; its a collection of highly optimised functions for creating webpages on a MicroPython device, such as the Raspberry Pi Pico W.

Here are some of the cool things it can do:

* Serve up Webpages
* Templating framework, (think Jinja)
* Logging
* Captive Portal / WiFi Access Point
* DNS Server

Check out the [Github](https://github.com/pimoroni/phew) page for more information.



---

## How to install Phew!

There are a couple of ways to install Phew! onto your Raspberry Pi Pico W - 

1. Download the code from <https://github.com/pimoroni/phew> and copy the files across [^1]
1. Use uPip to install, once you have established a wifi connection:

    ``` python
    import upip
    upip.install("micropython-phew")
    ```

> **NB** Phew is published to PyPi, making it easy to install, even within your code

---

## The rest of the code you'll probably need

To make the Access Portal do something useful, we need to capture all the requests from the guest computer and direct them to a route of our choice. This technique is call a ***Captive Portal***.

## DNS
We need to route any request for other websites and pages. We do this using our own DNS server; to launch this we simply add:

> `dns.run_catchall(<ip>)`

Here is a more fuller version of the code, for context: 

``` python
# Set to Accesspoint mode
ap = access_point("Wifi In The Woods")
ip = ap.ifconfig()[0]
logging.info(f"starting DNS server on {ip}")
dns.run_catchall(ip)
server.run()
logging.info("Webserver Started")
```

---

## A typical application

To further explain how to use this, consider the code below:

``` python

# CyberDog 
# Kevin McAleer
# September 2022

from phew import server, template, logging, access_point, dns
from phew.template import render_template
from phew.server import redirect
import gc
gc.threshold(50000) # setup garbage collection

DOMAIN = "pico.wireless" # This is the address that is shown on the Captive Portal

@server.route("/", methods=['GET','POST'])
def index(request):
    """ Render the Index page and respond to form requests """
    if request.method == 'GET':
        logging.debug("Get request")
        return render_template("index.html")
    if request.method == 'POST':
        text = request.form.get("text", None)
        logging.debug(f'posted message: {text}')
        return render_template("index.html", text=text)

@server.route("/wrong-host-redirect", methods=["GET"])
def wrong_host_redirect(request):
  # if the client requested a resource at the wrong host then present 
  # a meta redirect so that the captive portal browser can be sent to the correct location
  body = "<!DOCTYPE html><head><meta http-equiv=\"refresh\" content=\"0;URL='http://" + DOMAIN + "'/ /></head>"
  logging.debug("body:",body)
  return body

@server.route("/hotspot-detect.html", methods=["GET"])
def hotspot(request):
    """ Redirect to the Index Page """
    return render_template("index.html")

@server.catchall()
def catch_all(request):
    """ Catch and redirect requests """
    if request.headers.get("host") != DOMAIN:
        return redirect("http://" + DOMAIN + "/wrong-host-redirect")

# Set to Accesspoint mode
ap = access_point("Wifi In The Woods")  # Change this to whatever Wifi SSID you wish
ip = ap.ifconfig()[0]                   # Grab the IP address and store it
logging.info(f"starting DNS server on {ip}")
dns.run_catchall(ip)                    # Catch all requests and reroute them
server.run()                            # Run the server
logging.info("Webserver Started")


```

---

## Next Steps

Think about the possibilities of creating your own Access Points using the Pico W:

* Robot with a webpage accessible via the Captive Portal
* Find the Access Point in the Woods - a battery powered Pico W that shares some useful information in a remote location, maybe it gives clues for a GeoCache game.
* Setup impromptu Information points at temporary events

---

## Footnotes

[^1]: You can copy code using mpremote or Thonny