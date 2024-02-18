---
layout: lesson
title: Decrypting a message
author: Kevin McAleer
type: page
cover: /learn/octapi/assets/octapi.jpg
date: 2024-02-18
previous: 01_lesson01.html
next: 03_lesson03.html
description: null
percent: 42
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


## Decrypting a message

To decrypt a message using Py-Engima, you first need to create an Enigma machine. You can do this by creating an instance of the `EnigmaMachine` class and passing in the rotor settings and plugboard settings. Here's an example:

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

ciphertext = 'FKFPQZYVON'

plaintext = machine.process_text(ciphertext)
print(plaintext)
```

---

In this example, we create an instance of the `EnigmaMachine` class and pass in the rotor settings, reflector settings, ring settings, and plugboard settings. We then set the initial position of the rotors and use the `process_text` method to decrypt the message. The decrypted message is then printed to the console.

You can use the `process_text` method to decrypt any message that has been encrypted using the Enigma machine. You just need to create an instance of the `EnigmaMachine` class with the same rotor settings, reflector settings, ring settings, and plugboard settings that were used to encrypt the message. You also need to set the initial position of the rotors to the same position that they were in when the message was encrypted. Once you have done this, you can use the `process_text` method to decrypt the message.

---
