---
layout: lesson
title: Cracking Enigma with OctaPi
author: Kevin McAleer
type: page
cover: /learn/octapi/assets/octapi.jpg
date: 2024-02-18
previous: 05_dispy.html
description: Learn how to use DisPy and a Cluster of Raspberry Pis to perform a Bruteforce
  decryption.
percent: 100
duration: 4
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


## Cluster Setup

DisPy must be running on the Cluster nodes you wish to use.

I have created a simple [docker-compose](https://www.kevsrobots.com/learn/docker/) file to enable a quick setup. Click here to download the docker-compose & associated files: <https://github.com/kevinmcaleer/ClusteredPi/tree/main/stacks/octapi>.

The easiest way to setup DisPy is:

1. By cloning https://github.com/kevinmcaleer/ClusteredPi
1. Change to the octapi folder:

    ```bash
    cd ClusteredPi/stacks/octapi
    ```

1. Bring up the container:

    ```bash
    docker-compose up -d
    ```

    The docker image will be created and the container started with this single command.

1. To stop the container, simply type:

    ```bash
    docker-compose down
    ```

---

Run `02_bruteforce_octapi.py` to decrypt the message much quicker than running it stand-alone:

```python
import dispy

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

cribtext = "ROBOT"
text = "SUPSH GPYCV JTPYF TDQWV HIBEW FPDBN TAUEL IQXMS ZBDCT"
text = text.replace(" ","")

ciphertext = text
ring_choice = "1 1 1"

def decrypt_message(rotor_choice, start_pos, ciphertext):
    from enigma.machine import EnigmaMachine

    machine = EnigmaMachine.from_key_sheet(
    rotors=rotor_choice,
    reflector='B',
    ring_settings=ring_choice,
    plugboard_settings='AV BS CG DL FU HZ IN KM OW RX',
    )

    machine.set_display(start_pos)

    plaintext = machine.process_text(ciphertext).replace("X"," ")
    # print(plaintext)
    return (plaintext)

def find_rotor_start(rotor_choice:str,  ring_choice:str, cipher_text:str, crib_text:str):
    from enigma.machine import EnigmaMachine

    alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"

    machine = EnigmaMachine.from_key_sheet(
    rotors=rotor_choice,
    reflector='B', 
    ring_settings=ring_choice,
    plugboard_settings='AV BS CG DL FU HZ IN KM OW RX',
    )

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
                # if plaintext == crib_text:
                if crib_text in plaintext:
                    print("Valid settings found!")
                    print(rotor_choice, start_pos)
                    return rotor_choice, ring_choice, start_pos
         
    # If no valid settings are found, return None
    return rotor_choice, ring_choice, "Cannot find settings"

# nodes = ['192.168.2.2', '192.168.2.1', '192.168.2.4', '192.168.2.3']
nodes = ['192.168.2.*','192.168.1.*']
cluster = dispy.JobCluster(find_rotor_start, nodes=nodes, loglevel=dispy.logger.DEBUG)
print(f" cluster status {cluster.status()}")

jobs = []
id = 1
# Submit the jobs for this ring choice
for rotor_choice in rotors:
    job = cluster.submit(rotor_choice, ring_choice, ciphertext, cribtext)
    job.id = id # Associate an ID to the job
    jobs.append(job)
    id += 1   # Next job

print( "Waiting..." )
cluster.wait()
print( "Collecting job results" )

# Collect and check through the jobs for this ring setting
found = False

for job in jobs:
    # Wait for job to finish and return results

    if job.status == dispy.DispyJob.Finished:
        rotor_setting, ring_setting, start_pos = job()

    else:
        print(f"Job {job.id} did not finish, status is {job.status}")
        if job.exception:
            print(f"Job {job.id} failed with exception {job.exception}")

    if job.status == dispy.DispyJob.Finished:

        rotor_setting, ring_setting, start_pos = job()

        # If a start position was found
        if start_pos != "Cannot find settings":
            found = True
         
            message = decrypt_message(rotor_setting, start_pos, ciphertext)
            print(f"Rotors {rotor_setting}, ring {ring_setting}, message key was {start_pos}, using crib {cribtext}, message: {message}")

if found == False:
    print( 'Attack unsuccessful' )

cluster.print_status()
cluster.close()

```