---
layout: lesson
title: Encrypt longer messages
author: Kevin McAleer
type: page
cover: /learn/octapi/assets/octapi.jpg
date: 2024-02-18
previous: 02_lesson02.html
next: 04_lesson04.html
description: null
percent: 56
duration: 2
navigation:
- name: OctaPi
- content:
  - section: Overview
    content:
    - name: Introduction
      link: 00_intro.html
  - section: Introduction to Enigma
    content:
    - name: Encrypt
      link: 01_lesson01.html
    - name: Decrypting a message
      link: 02_lesson02.html
  - section: Bruteforce attack
    content:
    - name: Encrypt longer messages
      link: 03_lesson03.html
    - name: Bruteforce descryption
      link: 04_lesson04.html
  - section: Build OctaPi
    content:
    - name: DisPy
      link: 05_dispy.html
    - name: Cracking Enigma with OctaPi
      link: 06_octapi.html
---


Learn how to encrypt messages like the one below using the Enigma machine and Py-Engima.

`HEY ROBOT MAKERS THIS IS A SECRET MESSAGE` becomes:

`SUPSH GPYCV JTPYF TDQWV HIBEW FPDBN TAUEL IQXMS ZBDCT`

> ## No Spaces
>
> The Enigma machine had no spacebar, so spaces were represented by the letter "X". This means that the message above should be read as:
>
> `HEYXROBOTXMAKERSXTHISXISXAXSECRETXMESSAGE'
>
> We can easily replace space characters using the python .replace() method.
>
> e.g.
>
> ```python
> plaintext = plaintext.replace(" ","X")
> ```
>

---

```python
from enigma.machine import EnigmaMachine

machine = EnigmaMachine.from_key_sheet(
   rotors='II V III',
   reflector='B',
   ring_settings='1 1 1',
   plugboard_settings='AV BS CG DL FU HZ IN KM OW RX',
   )

# Set the initial position of the Enigma rotors
machine.set_display('QJF')

def pad_string(input):
    padding_needed = (5 - len(input) % 5) % 5
    # Append the "X"s to the end of the string
    padded_string = input + "X" * padding_needed
    return padded_string

def format_for_transmission(input_string):
    # Use list comprehension to insert a space after every 5th character
    # [input_string[i:i+5] for i in range(0, len(input_string), 5)] creates substrings of every 5 characters
    # ' '.join(...) then joins these substrings with a space

    # Calculate the number of "X"s needed to make the length divisible by 5
    
    return ' '.join(input_string[i:i+5] for i in range(0, len(input_string), 5))

plaintext = 'HEY ROBOT MAKERS THIS IS A SECRET MESSAGE'
plaintext = plaintext.replace(" ","X")
plaintext = pad_string(plaintext)
ciphertext = machine.process_text(plaintext)

print(format_for_transmission(ciphertext))

# The output should be:
# SUPSH GPYCV JTPYF TDQWV HIBEW FPDBN TAUEL IQXMS ZBDCT
```

---

In the next lesson, we will learn how to use Py-Engima to crack the Enigma machine using a brute force attack.

---