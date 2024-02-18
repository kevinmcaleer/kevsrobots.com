---
title: Bruteforce descryption
description: Learn how to crack the Enigma machine using a brute force attack
layout: lesson
---

## What is a brute force attack?

A brute force attack is a method of cracking a code by trying every possible combination of characters until the correct one is found. This is a very time-consuming process, but it is guaranteed to work eventually.

---

## Bruteforce attack on the Enigma machine

To crack the Enigma machine using a brute force attack, we need to try every possible combination of rotor settings and start positions until we find the correct one. This is a very time-consuming process, as there are 60^3 = 216,000 possible combinations of rotor settings and start positions.

To understand when we have found the correct settings, we can use a small substring of the plaintext message, known as a "crib". This is a word or phrase that we know is in the plaintext message. If we find the crib in the decrypted message, we know that we have found the correct settings.

---

```python
import time

# ciphertext = "YJPYITREDSYUPIU"
cribtext = "ROBOT"
message = "SUPSH GPYCV JTPYF TDQWV HIBEW FPDBN TAUEL IQXMS ZBDCT"

rotors = [ "I II III", "I II IV", "I II V", "I III II",
"I III IV", "I III V", "I IV II", "I IV III",
"I IV V", "I V II", "I V III", "I V IV",
"II I III", "II I IV", "II I V", "II III I",
"II III IV", "II III V", "II IV I", "II IV III",
"II IV V", "II V I", "II V III", "II V IV",
"III I II", "III I IV", "III I V", "III II I",
"III II IV", "III II V", "III IV I", "III IV II",
"III IV V", "IV I II", "IV I III", "IV I V",
"IV II I", "IV II III", "IV I V", "IV II I",
"IV II III", "IV II V", "IV III I", "IV III II",
"IV III V", "IV V I", "IV V II", "IV V III",
"V I II", "V I III", "V I IV", "V II I",
"V II III", "V II IV", "V III I", "V III II",
"V III IV", "V IV I", "V IV II", "V IV III" ]

plugboard = "AV BS CG DL FU HZ IN KM OW RX"
slip_ring = "1 1 1"

def decrypt_message(rotor_choice, start_pos, ciphertext):
    from enigma.machine import EnigmaMachine

    machine = EnigmaMachine.from_key_sheet(
    rotors=rotor_choice,
    reflector='B',
    ring_settings=slip_ring,
    plugboard_settings=plugboard)

    machine.set_display(start_pos)

    plaintext = machine.process_text(ciphertext)
    print(plaintext.replace("X"," "))


def find_rotor_start(rotor_choice, cipher_text, crib_text):
    from enigma.machine import EnigmaMachine

    alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"

    machine = EnigmaMachine.from_key_sheet(
    rotors=rotor_choice,
    reflector='B',
    ring_settings=slip_ring,
    plugboard_settings=plugboard)

    for rotor1 in alphabet:
        for rotor2 in alphabet:
            for rotor3 in alphabet:
                
                # Generate a possible start position
                start_pos = rotor1 + rotor2 + rotor3
                
                # Set the start position
                machine.set_display(start_pos)

                # Attempt to decrypt the plaintext
                plaintext = machine.process_text(cipher_text)
                print(plaintext)
                if crib_text in plaintext:
                    print("Valid settings found! *************************************************")
                    decrypt_message(rotor_choice, start_pos, message)
                    return rotor_choice, start_pos
         
    # If no valid settings are found, return None
    return rotor_choice, "Cannot find settings"

message = message.replace(" ","")

start_time = time.time()

for rotor_setting in rotors:
    rotor_choice, start_pos = find_rotor_start(rotor_setting, message, cribtext)
    print(rotor_choice, start_pos)
    if start_pos != "Cannot find settings":
        break

end_time = time.time()

duration = end_time - start_time

print(f"Brute force attack completed in {round(duration,1)} seconds on Mac M1")
```

---
