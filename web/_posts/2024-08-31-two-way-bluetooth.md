---
title: "Two-Way Bluetooth Communication Between Raspberry Pi Picos"
description: >-
    Learn how to set up two-way Bluetooth communication between two Raspberry Pi Picos using MicroPython.
excerpt: >-
layout: showcase
date: 2024/08/31
author: Kevin McAleer
difficulty: beginner
cover: /assets/img/blog/pico2pico/cover.png
hero:  /assets/img/blog/pico2pico/hero.png
mode: light
tags: 
 - Raspberry Pi
 - Pico
 - Bluetooth
groups:
 - raspberrypi
 - pico
 - micropython
videos:
 - Ln_xreldWOM
code:
 - https://www.github.com/kevinmcaleer/pico_two_way_bluetooth
---


In this tutorial, we’ll walk through setting up `two-way Bluetooth` communication between two Raspberry Pi Picos. We’ll create two programs—one for each Pico—that will allow them to both send and receive messages over Bluetooth. This will demonstrate how to set up basic full-duplex communication between microcontrollers, which can be a powerful tool for IoT projects and device-to-device communication.

---

## Prerequisites

Before we dive into the code, make sure you have the following:

- Two Raspberry Pi Picos.
- Bluetooth modules compatible with the `aioble` library (e.g., the Pico W with built-in Wi-Fi and Bluetooth).
- A development environment set up for MicroPython on the Picos.
- The `aioble` and `uasyncio` libraries installed on both Picos.

---

### Step 1: Setting Up the Bluetooth Service and Characteristics

Both Picos will use the same Bluetooth service and characteristic UUIDs. These UUIDs uniquely identify the service and characteristic that we will use for communication.

```python
import aioble
import bluetooth
import uasyncio as asyncio

# Define UUIDs for the service and characteristic
_SERVICE_UUID = bluetooth.UUID(0x1848) 
_CHARACTERISTIC_UUID = bluetooth.UUID(0x2A6E)
```

The `_SERVICE_UUID` is used to identify the Bluetooth service that we’ll use for communication. The `_CHARACTERISTIC_UUID` identifies the specific characteristic within that service where data will be read from or written to.

---

### Step 2: Programming Pico A (Sender & Receiver)

Pico A will act as both a Bluetooth server and client. It will advertise itself as "PicoA" and wait for connections. Once connected, it will start sending and receiving messages.

```python
async def send_data_task(connection, characteristic):
    while True:
        message = "Hello from Pico A!"
        await characteristic.write(message.encode(), response=True)
        print(f"Pico A sent: {message}")
        await asyncio.sleep(2)  # Wait for 2 seconds before sending the next message
```

In the `send_data_task` function, Pico A sends a message every 2 seconds. It writes this message to the characteristic, which the connected device (Pico B) will read.

```python
async def receive_data_task(connection, characteristic):
    while True:
        try:
            data = await characteristic.notified()
            print(f"Pico A received: {data.decode()}")
        except asyncio.TimeoutError:
            print("Timeout waiting for data in Pico A.")
            break
```

The `receive_data_task` function continuously waits for notifications on the characteristic. When data is received, it prints the message.

```python
async def run_pico_a():
    # Set up the Bluetooth service and characteristic
    service = aioble.Service(_SERVICE_UUID)
    characteristic = aioble.Characteristic(service, _CHARACTERISTIC_UUID, read=True, notify=True)

    # Start advertising
    aioble.advertise("PicoA", services=[service])
    print("Pico A is advertising...")

    async with aioble.gap_connect(service) as connection:
        print("Pico A connected to another device.")

        # Create tasks for sending and receiving data
        tasks = [
            asyncio.create_task(send_data_task(connection, characteristic)),
            asyncio.create_task(receive_data_task(connection, characteristic)),
        ]
        
        await asyncio.gather(*tasks)
```

In the `run_pico_a` function, Pico A advertises itself as "PicoA" and waits for a connection. Once a connection is established, it starts both sending and receiving data.

```python
async def main():
    await run_pico_a()

asyncio.run(main())
```

The `main` function simply runs the `run_pico_a` function, which starts the entire process.

---

### Step 3: Programming Pico B (Sender & Receiver)

Pico B will scan for Pico A and, once found, connect to it and begin sending and receiving data, similar to Pico A.

```python
async def send_data_task(connection, characteristic):
    while True:
        message = "Hello from Pico B!"
        await characteristic.write(message.encode(), response=True)
        print(f"Pico B sent: {message}")
        await asyncio.sleep(2)  # Wait for 2 seconds before sending the next message
```

The `send_data_task` in Pico B is identical to Pico A’s, but it sends a different message: "Hello from Pico B!".

```python
async def receive_data_task(connection, characteristic):
    while True:
        try:
            data = await characteristic.notified()
            print(f"Pico B received: {data.decode()}")
        except asyncio.TimeoutError:
            print("Timeout waiting for data in Pico B.")
            break
```

The `receive_data_task` also mirrors Pico A’s, listening for data from Pico A and printing it when received.

```python
async def run_pico_b():
    # Scan for 5 seconds to find the target device (Pico A)
    async with aioble.scan(5000, interval_us=30000, window_us=30000, active=True) as scanner:
        async for result in scanner:
            print(f"Found device: {result.name()}")
            if result.name() == "PicoA":
                print("Found PicoA, attempting to connect...")
                device = result.device
                break
        else:
            print("PicoA not found.")
            return

    try:
        # Connect to the device
        connection = await device.connect()
        async with connection:
            print("Connected to PicoA")

            # Find the service and characteristic on the receiver
            service = await connection.service(_SERVICE_UUID)
            characteristic = await service.characteristic(_CHARACTERISTIC_UUID)

            # Create tasks for sending and receiving data
            tasks = [
                asyncio.create_task(send_data_task(connection, characteristic)),
                asyncio.create_task(receive_data_task(connection, characteristic)),
            ]
            
            await asyncio.gather(*tasks)

    except Exception as e:
        print(f"Error during connection or communication: {e}")
```

In `run_pico_b`, Pico B scans for nearby Bluetooth devices, looking specifically for "PicoA". Once found, it connects and starts sending and receiving data.

```python
async def main():
    await run_pico_b()

asyncio.run(main())
```

Again, the `main` function simply runs the `run_pico_b` function to kickstart the process.

---

### Step 4: Running the Programs

1. **Flash the Pico A program onto one of your Picos.** This device will advertise itself as "PicoA" and wait for connections.
2. **Flash the Pico B program onto the other Pico.** This device will scan for "PicoA", connect to it, and begin the two-way communication.
3. **Monitor the outputs:** You should see "Hello from Pico A!" and "Hello from Pico B!" messages being printed alternately on both Picos.

---

## Conclusion

In this tutorial, we set up two-way Bluetooth communication between two Raspberry Pi Picos. Both Picos acted as servers and clients, allowing them to send and receive data simultaneously. This setup can be the foundation for more complex IoT projects where devices need to exchange data or commands in real-time.

By understanding the basics of Bluetooth communication in MicroPython, you can now extend this knowledge to create even more advanced applications, such as remote control systems, sensor networks, or even simple chat systems between devices.

Happy coding!

---
