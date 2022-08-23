---
layout: blog
title: PIKON Camera - Raspberry Pi High Quality Camera Project
date: 2022-08-23
author: Kevin McAleer
excerpt: Build your own portable Raspberry Pi Camera featuring the High Quality
cover: /assets/img/blog/pikon/pikon.jpg
tags:
 - Raspberry Pi
 - Pikon
 - Camera
 - High Quality Camera
 - Python
 
---

## Table of Contents

{:toc}
* toc

---

## PIKON Camera Project
During the recent [Pimoroni Sale](https://www.pimoroni.com) I purchased an official [Raspberry Pi High Quality Camera](https://shop.pimoroni.com/products/raspberry-pi-high-quality-camera). I wanted to create a suitable project to showcase the camera, and as a [livestreamer](https://www.youtube.com/c/kevinmcaleer28) I look at a DSLR camera everyday, so I decided to recreate a DSLR shaped body in Fusion 360.

The High Quality Camera Module has 4 mount points, so the first task is to model something that it can attach to.

---

### Dimensions
![Module Dimensions](/assets/img/blog/pikon/module_dimensions.png){:class="img-fluid w-50"}

The diagram above shows the main dimensions of the module; the mounting holes are 2.5mm in diameter, the holes are spaced 30mm from the center point.

So the first step is to create a flat section to mount this too. I like to use 3mm as the thickness of walls or plates as it is quick to print and makes for solid, sturdy construction. We'll need to extrude some stand offs for the mount points to allow for any components that are not quite flush on the backside of the Camera Mount. 3mm should do the trick.

![Mounting Plate](/assets/img/blog/pikon/mounting_plate.png){:class="img-fluid w-50"}

The next main set of dimensions we need are for the Raspberry Pi 2/3/4. 

![Raspberry Pi Dimensions](/assets/img/blog/pikon/pi_dimensions.png){:class="img-fluid w-50"}

The Pi's mounting points are the same 2.75mm holes, spaced 58mm and 49mm from the center points.

We could just call it quits at this point, but it would be nice to house a battery, powerful enough to run the Raspberry Pi for several hours.

---

### Battery

I love the [NanoWave 3 5000mAh USB-C & A Power Bank](https://shop.pimoroni.com/products/nanowave-3-5000mah-usb-c-a-power-bank), it has a USB C, and USB A connector for powering your projects. The 18650 within it is a 5000mah, which should keep our Pi running for a couple of hours.

The battery can be contained within our DSLR, within the hand grip. I quite like the steam-punk aesthetic so I've chosen to have the battery connect to the Raspberry Pi using a USB cable that comes out of the case and back into the case. This also makes charging the Battery, or powering the Raspberry Pi externally much easier.

![Battery](/assets/img/blog/pikon/battery.png){:class="img-fluid w-50"}

---

### Display
Finally, any DSLR camera wouldn't be complete without a nice display, and I just happened to have some old Waveshare 3.5" displays to hand, which connect to the Raspberry Pi via the 40 pin header.

![Display](/assets/img/blog/pikon/display.png){:class="img-fluid w-50"}

---

### Lenses
The Raspberry Pi High Quality Camera Module doesn't come with a lens, and there are many to choose from. I originally went with the cheapest one, which was only £9 in the Pimoroni sale, however its wasn't a great choice for general use, so I've since purchased the 16mm (telephoto) and and 6mm (wide angle) versions.

---

### Putting it all together

The components are pretty simple to put together and only require some M2 Nuts and bolts to hold everything together.

The bottom section is attached with 2 x M2 screws.

### STL files
If you want to download the project files for this, they are available here:

* [Chassis](/assets/stl/pikon/chassis.stl)
* [Bottom](/assets/stl/pikon/bottom.stl)
* [Shutter Button](/assets/stl/pikon/shutter_button.stl)

![3d Render](/assets/img/blog/pikon/final.png){:class="img-fluid w-50"}

![photo](/assets/img/blog/pikon/pikon.jpg){:class="img-fluid w-50"}

---

### Next Steps

This is the hardware build complete, I'm intending to build out a Python App that can be run on the Raspberry Pi 4 that will be able to record video clips, photos and apply filters. With this being a full blown Raspberry Pi 4, we can also livestream from this camera too.
