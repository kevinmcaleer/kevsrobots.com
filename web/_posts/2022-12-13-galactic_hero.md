---
layout: blog
title: Galactic Hero Game
short_title: Galactic Hero Game
short_description: Build your own Guitar Hero Clone
description: Build your own Guitar Hero Clone
date: 2022-12-13
author: Kevin McAleer
excerpt: Learn how to build your own Guitar Hero Clone with the Pimoroni Galactic Unicorn and MicroPython
cover: /assets/img/blog/galactic_hero/galactic_hero01.jpg
tags:
 - Raspberry Pi Pico
 - MicroPython
 - Pimoroni
 - Galactic Unicorn
 - Game
groups:
 - games
 - micropython
 - pico
---

## Contents

{:toc}
* toc

---

## YouTube Video

Click the thumbnails below to watch the show all about this build.

{% include youtubeplayer.html id="OBAn3GVU4v0" %}

## Overview

Build your own Galactic Guitar Hero game with Pimoroni Galactic Unicorn (GU) and some MicroPython code. It's fun and easy to enhance too.

---

## Bill of Materials

For this project you'll just need a Pimoroni Galactic Unicorn display. It features 153 pixel display, along with a Raspberry Pi Pico W aboard.

Item             | Description               | Qty |  Price
-----------------|---------------------------|----:|------:
Galactic Unicorn | [Pimoroni Galactic Unicorn](https://shop.pimoroni.com/products/galactic-unicorn) |   1 | £49.50
{:class="table table-striped"}

---

## The Galactic Unicorn

Here are some of the main features of the Pimoroni Galactic Unicorn:

* Raspberry Pi Pico W Aboard
* 583 RGB LEDs in a 53 x 11 grid
* 3.5mm LEDs with rounded square apertures
* 6mm LED spacing
* Driven by 10 FM6047 constant current LED drivers
* MAX98357 3.2W I2S Mono Amplifier (with 30mm 1W speaker)
* Phototransistor for light sensing
* 9 tactile user buttons
* Reset button
* 2x Qw/ST (Qwiic/STEMMA QT) connectors
* JST-PH connector for attaching a battery (5.5V max)
* Fully assembled
* No soldering required#

![Front view of the Pimoroni Galactic Unicorn](/assets/img/blog/galactic_hero/slide01.jpg){:class="img-fluid w-100"}

![Front view of the Pimoroni Galactic Unicorn](/assets/img/blog/galactic_hero/slide02.jpg){:class="img-fluid w-100"}

---

## Build your own Galactic Hero game with Pimoroni Galactic Unicorn

### Gameplay - How to win or loose

Galactic Hero gameplay is simliar to the original Guitar Hero:

* Notes fall down the guitar neck towards the scoring line
* The player presses the corresponding button for each column of notes as it passes the scoring line
* The player looses a life with each note that is missed
* Game stops when all lives are lost

![Front view of the Pimoroni Galactic Unicorn](/assets/img/blog/galactic_hero/slide03.jpg){:class="img-fluid w-100"}

The scoring line is a white line on column 50 of the display, and each of the five columns (guitar frets) has a corresponding button at the bottom of the GU guitar neck.

![Front view of the Pimoroni Galactic Unicorn](/assets/img/blog/galactic_hero/slide04.jpg){:class="img-fluid w-100"}

---

### GPIO Pinouts

The GU MicroPython library has a number of constants to help with writing programs. Each of the nine user programmable buttons has a GPIO pin associated with it, and we will use just five of these in our game:

GPIO Pin | Galactic Unicorn Function | Galactic Guitar Hero Assignment
:-:|--:|--:
26 | SWITCH_BRIGHTNESS_DOWN | button_a
21 | SWITCH_BRIGHTNESS_UP | button_b
27 | SWITCH_SLEEP | button_c
8 | SWITCH_VOLUME_DOWN | button_d
7 |SWITCH_VOLUME_UP | button_e
0 |SWITCH_A | -
1 | SWITCH_B | -
3 | SWITCH_C | -
6| SWITCH_D | -
{:class="table table-striped"}

---

### Game Notes

The notes for the game are simply stored in a `tuple`. The notes are represented in Python in the way they will appear on screen, with a `1` representing a note for that fret position, and a `0` meaning no note.

To keep the code in the main program clean, the game notes are stored in a separate file: `tune01.py`, which must be copied to the GU; this can be easily done with Thonny (right click on the file and click `upload to /`).

```python
tune = ('00001',
        '00010',
        '00100',
        '01000',
        '10000',
        '01000',
        '00100',
        '00010',
        '00001',
        '00010',
        '00100',
        '01000',
        '10000',
        '01000',
        '00100',
        '00010',
        '00001',
        )
```

---

### PicoGraphics

We can use the excellent Pimoroni `PicoGraphics` MicroPython library to display our notes on the display. For this we will need to use three different functions.

![Front view of the Pimoroni Galactic Unicorn](/assets/img/blog/galactic_hero/slide07.jpg){:class="img-fluid w-100"}

We can create colour pens using the function:

```python
display.create_pen(red, green, blue)
```

`create_pen` is used to create a pen object that holds a set of colours. Pens are used by PicoGraphics to draw coloured pixels on displays.

We can set colours using the function:

``` python
display.set_pen(pen)
```

`set_pen` is used to set the pen with a colour and the next graphics operation (such as `pixel`) will use this colour.

We can draw pixels using the function: 

```python
display.pixel(x, y)
```

`pixel` actually lights up the pixel at the `x` and `y` position on the display, using the current `pen` colour.

---

## MicroPython code

### The main game loop

Below is the main game loop, we'll look through each step to understand what is going on:

```python
while lives >= 1 :
    display.set_pen(black)
    display.clear()
    display_board()
    display_tune(tune,x,y)
    check_missed()
    winning = check_buttons()
    if not winning:
        lives -= 1
        print('lost a life, lives remaining:',lives)
    x = x + 1
    gu.update(display)
    sleep(0.01)
    offset = x - len(tune[0])
    if offset > WIDTH-4:
        x = x_reset
    
print("you lost")
```


`while lives >= 1`                             | this means keep looping round until the `lives` variable is greater or equal to `1`, when we have `0` lives the game will end.
`display.set_pen(black)`                       | sets the current pen colour to `black`
`display.clear()`                              | sets all the pixels on the display to the current pen colour, which is `black`
`display_board()`                              | displays the game board, we'll look at this below
`display_tune(tune,x,y)`                       | displays the notes at the `x` and `y` position on the display, only the `x` position changes, as the notes *fall* down the display
`check_missed()`                               | this is a function that checks to see if any notes have been missed and changes the score line to `red`.
`winning = check_buttons()`                    | this is a function that checks to see if the player has pressed the corresponding buttons for any frets with notes in the scoring zone, if they have `winning` is `True` if they haven't winning is `False`
`if not winning:`                              | if the `winning` variable is not `True` then run the code block below
`lives -= 1`                                   | take 1 life away from the play
`print('lost a life, lives remaining:',lives)` | print the message to the console `lost a life, lives remaining` and the current value of `lives`
`x = x + 1`                                    | increment the x position, so that the next time the notes are drawn, they appear to fall down a row
`gu.update(display)`                           | update the display - this will make all the pixels appear in their new position
`sleep(0.01)`                                  | sleep for 100th of a second.
`offset = x - len(tune[0])`                    | update the `offset` to equal the current `x` value minus the lenth of the `tune`; we only need to check one row which is what `tune[0]` means - check the zeroth row length
`if offset > WIDTH-4:`                         | check if the value of `offset` is greater than the value of (`WIDTH` minus `4`)
`x = x_reset`                                  | reset the `x` position back to the value stored in `x_reset` - this makes the notes appear just off the top of the display ready to drop down on the new loop cycle
`print("you lost")` | Print the message `you lost` to the console
{:class="table table-code"}

---

### display_board()

The `display_board` function displays the game board, which is just a white line at row `50` on the display.

```python
def display_board():
    """ Display the gameboard """
    display.set_pen(white)
    display.line(50,0,50,11)
    display.update()
```

---

### display_tune(tune,x,y)

The `display_tune(tune,x,y)` function takes three parameters

* the `tuple` containin the notes
* the `x` position to display the notes
* the `y` position to displat the notes

```python
def display_tune(bitmap, x:int, y:int):
    """ Display a tune on the screen at x,y coordinates """
    global fret_a, fret_b, fret_c, fret_d, fret_e
    row_offset = 0
    col_offset = 0
    fret_a = False
    fret_b = False
    fret_c = False
    fret_d = False
    fret_e = False
    # Loop through each row of the bitmap
    for row in bitmap:
        
        row_offset = 0
        fret_no = 0
        # Loop through each pixel in the row
        for pixel in row:
            
            # check if row is on screen, within the bounds of the display
            if len(row)-row_offset < WIDTH:
                if len(bitmap)+(col_offset//2) < HEIGHT:
                    if pixel == '1':
                        colour = fret_colours[col_offset//2]
                        
                        display.set_pen(colour)
                        
                        # set the game state
                        if x+row_offset == 50:
                            if y+col_offset == 0: fret_e = True
                            if y+col_offset == 2: fret_d = True
                            if y+col_offset == 4: fret_c = True
                            if y+col_offset == 6: fret_b = True
                            if y+col_offset == 8: fret_a = True
                    else:
                        
                        display.set_pen(black)

                    # display the pixel, double width
                    display.pixel(x+row_offset, y+col_offset)
                    display.pixel(x+row_offset, y+col_offset+1)
                        
            fret_no += 1
            row_offset += 2 
        col_offset += 2
```

`global fret_a, fret_b, fret_c, fret_d, fret_e` | this allows the `fret_a` thorugh to `fret_e` global variables to be changed, normally any changes would only be made local to the function. The `fret_x` variables hold a `True` if a note is currently in the scoring zone, and `False` if not
`row_offset = 0` | set `row_offset` to zero
`col_offset = 0` | set `col_offset` to zero
`fret_a = False` | set `fret_a` to False - this represents if a note on Fret a is in the scoring zone
`fret_b = False` | set `fret_b` to False - this represents if a note on Fret b is in the scoring zone
`fret_c = False` | set `fret_c` to False - this represents if a note on Fret c is in the scoring zone
`fret_d = False` | set `fret_d` to False - this represents if a note on Fret d is in the scoring zone
`fret_e = False` | set `fret_e` to False - this represents if a note on Fret e is in the scoring zone
`for row in bitmap:` | for each `row` of the `bitmap`, run the code block below
`row_offset = 0` | reset the `row_offset` to zero
`fret_no = 0` | set the current `fret_no` (fret number) to zero
`for pixel in row:` | for each `pixel` in the current `row`, run the code block below
`if len(row)-row_offset < WIDTH:` | if the length of the current `row` minus the `row_offset` is less than the display `WIDTH`, run the code block below
`if len(bitmap)+(col_offset//2) < HEIGHT:` | if the length the of the `bitmap` plus the `col_offset` divided by 2 is less than the display `WIDTH`, run the code block below
`if pixel == '1':` | if the current `pixel` is the value `1`, run the code block below
`colour = fret_colours[col_offset//2]` | set the current colour to the correct colour for this fret, (col_offset divided by two is because we are stretching pixels twice the width above)
`display.set_pen(colour)` |  Set the pen colour to the fret colour.
{:class="table table-code"}

---

### check_buttons

```python
def check_buttons():
    """ Check the buttons on the Galactic Unicorn """
    win = True
    button_a = GalacticUnicorn.SWITCH_BRIGHTNESS_DOWN
    button_b = GalacticUnicorn.SWITCH_BRIGHTNESS_UP
    button_c = GalacticUnicorn.SWITCH_SLEEP
    button_d = GalacticUnicorn.SWITCH_VOLUME_DOWN
    button_e = GalacticUnicorn.SWITCH_VOLUME_UP
        
    tests = {fret_a:button_a, fret_b:button_b, fret_c:button_c, fret_d:button_d, fret_e:button_e}

    for fret, button in tests.items():
        if fret:
            if gu.is_pressed(button):
                win = True
            else:
                win = False
                break
    return win
```

`win = True` | Set the `win` to True; this is a flag to store the winning state
`button_a = GalacticUnicorn.SWITCH_BRIGHTNESS_DOWN` | set button_a to the GU Brightness Down button
`button_b = GalacticUnicorn.SWITCH_BRIGHTNESS_UP` | set button_b to the GU Brightness Up button
`button_c = GalacticUnicorn.SWITCH_SLEEP` | set button_c to the GU Sleep button
`button_d = GalacticUnicorn.SWITCH_VOLUME_DOWN` | set button_d to the GU Volumne Down button
`button_e = GalacticUnicorn.SWITCH_VOLUME_UP` | set button_e to the GU Volume Up button
`tests = {fret_a:button_a, fret_b:button_b, fret_c:button_c, fret_d:button_d, fret_e:button_e}` | create a list of fret states and matching buttons
`for fret, button in tests.items():` | for each fret and button, loop through items in tests list and run the code block below
`if fret:` | if the current fret is `True`, run the code block below
`if gu.is_pressed(button):` | if the current fret button is pressed, run the code block below
`win = True` | set the win flag to `True`
`else:` | otherwise
`win = False` | set the win flag to `False`
`break` | break out of the loop
`return win` | return the current win flag status `True` or `False`
{:class="table table-code"}

---

### check_missed

```python
def check_missed():
    """ Check if the note passed the bridge without a button being pressed """
    if fret_a or fret_b or fret_c or fret_d or fret_e:
        display.set_pen(red)
        display.rectangle(50,0,1,11)
        return True
    else:
        return False
```

`if fret_a or fret_b or fret_c or fret_d or fret_e:` | if any of the frets are true; that is there is a note in the scoring of each fret
`display.set_pen(red)` | set the pen colour to red
`display.rectangle(50,0,1,11)` | display a red line on the scoring line
`return True` | return `True`
`else:` |  otherwise
`return False` | return `False`
{:class="table table-code"}

---

### Full Code listing 

```python
# Galactic Hero - A game for the Pimoroni Pico Display
# Kevin McAleer
# December 2022

from picographics import PicoGraphics, DISPLAY_GALACTIC_UNICORN as DISPLAY
from time import sleep
from galactic import GalacticUnicorn

# Set up the display
gu = GalacticUnicorn()
display = PicoGraphics(display=DISPLAY)
gu.set_brightness(0.25)
WIDTH, HEIGHT = display.get_bounds()

# Set up the colours
GREEN = {'red':0,'green':255,'blue':0}
RED = {'red':255,'green':0,'blue':0}
YELLOW = {'red':255,'green':255,'blue':0}
CYAN = {'red':0,'green':255,'blue':255}
ORANGE = {'red':255,'green':128,'blue':0}
BLACK = {'red':0,'green':0,'blue':0}
WHITE = {'red':255,'green':255, 'blue':255}

def create_pen(display, color):
    """ Create a pen from a colour dictionary """
    return display.create_pen(color['red'],color['green'],color['blue'])

# Create the pens
red = create_pen(display, RED)
green = create_pen(display, GREEN)
yellow = create_pen(display, YELLOW)
cyan = create_pen(display, CYAN)
orange = create_pen(display, ORANGE)
black = create_pen(display, BLACK)
white = create_pen(display, WHITE)

# Set the fret colours

fret_colours = {0:orange, 1:cyan, 2:yellow, 3:green, 4:red}

# Set the game button states
fret_a = False
fret_b = False
fret_c = False
fret_d = False
fret_e = False
winning = True

# Create the tune pattern - moved to a file
from tune01 import tune

def transpose(bitmap):
    """ Transpose a bitmap (This is because the Galactic Unicorn is rotated 90 degrees) """
    transposed_bitmap = [''.join([string[i] for string in bitmap]) for i in range(len(bitmap[0]))]
    return transposed_bitmap

def display_tune(bitmap, x:int, y:int):
    """ Display a tune on the screen at x,y coordinates """
    global fret_a, fret_b, fret_c, fret_d, fret_e
    row_offset = 0
    col_offset = 0
    fret_a = False
    fret_b = False
    fret_c = False
    fret_d = False
    fret_e = False
    # Loop through each row of the bitmap
    for row in bitmap:
        
        row_offset = 0
        fret_no = 0
        # Loop through each pixel in the row
        for pixel in row:
            
            # check if row is on screen, within the bounds of the display
            if len(row)-row_offset < WIDTH:
                if len(bitmap)+(col_offset//2) < HEIGHT:
                    if pixel == '1':
                        colour = fret_colours[col_offset//2]
                        
                        display.set_pen(colour)
                        
                        # set the game state
                        if x+row_offset == 50:
                            if y+col_offset == 0: fret_e = True
                            if y+col_offset == 2: fret_d = True
                            if y+col_offset == 4: fret_c = True
                            if y+col_offset == 6: fret_b = True
                            if y+col_offset == 8: fret_a = True
                    else:
                        
                        display.set_pen(black)

                    # display the pixel, double width
                    display.pixel(x+row_offset, y+col_offset)
                    display.pixel(x+row_offset, y+col_offset+1)
                        
            fret_no += 1
            row_offset += 2 
        col_offset += 2
        
def display_board():
    """ Display the gameboard """
    display.set_pen(white)
    display.line(50,0,50,11)
    display.update()
        
def fret_debug():
    if fret_a:
        display.set_pen(yellow)
        display.pixel(0,0)
    else:
        display.set_pen(black)
        display.pixel(0,0)
    if fret_b:
        display.set_pen(yellow)
        display.pixel(1,0)
    else:
        display.set_pen(black)
        display.pixel(1,0)
    if fret_c:
        display.set_pen(yellow)
        display.pixel(2,0)
    else:
        display.set_pen(black)
        display.pixel(2,0)
    if fret_d:
        display.set_pen(yellow)
        display.pixel(3,0)
    else:
        display.set_pen(black)
        display.pixel(3,0)
    if fret_e:
        display.set_pen(yellow)
        display.pixel(4,0)
    else:
        display.set_pen(black)
        display.pixel(4,0)
    display.update()
        
def check_buttons():
    """ Check the buttons on the Galactic Unicorn """
    win = True
    button_a = GalacticUnicorn.SWITCH_BRIGHTNESS_DOWN
    button_b = GalacticUnicorn.SWITCH_BRIGHTNESS_UP
    button_c = GalacticUnicorn.SWITCH_SLEEP
    button_d = GalacticUnicorn.SWITCH_VOLUME_DOWN
    button_e = GalacticUnicorn.SWITCH_VOLUME_UP
        
    tests = {fret_a:button_a, fret_b:button_b, fret_c:button_c, fret_d:button_d, fret_e:button_e}

    for fret, button in tests.items():
        if fret:
            if gu.is_pressed(button):
                win = True
            else:
                win = False
                break
    return win

def check_missed():
    """ Check if the note passed the bridge without a button being pressed """
    if fret_a or fret_b or fret_c or fret_d or fret_e:
        display.set_pen(red)
        display.rectangle(50,0,1,11)
        return True
    else:
        return False

# Transpose the tune
tune = transpose(tune)

# Set the starting position of the tune
x_reset = -len(tune[0]*2)
x = x_reset
y = 0

winning = True
lives = 3
print(f'Galactic Hero')

while lives >= 1 :
    display.set_pen(black)
    display.clear()
    display_board()
    display_tune(tune,x,y)
    check_missed()
    winning = check_buttons()
    if not winning:
        lives -= 1
        print('lost a life, lives remaining:',lives)
    x = x + 1
    gu.update(display)
    sleep(0.01)
    offset = x - len(tune[0])
    if offset > WIDTH-4:
        x = x_reset
    
print("you lost")
```