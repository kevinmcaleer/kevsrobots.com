---
title: "Two-Way Bluetooth Communication Between Raspberry Pi Picos"
description: >-
    Learn how to set up two-way Bluetooth communication between two Raspberry Pi Picos using MicroPython.
excerpt: >-
layout: showcase
date: 2024-08-31
author: Kevin McAleer
difficulty: beginner
cover: /assets/img/blog/pico2pico/cover.png
hero:  /assets/img/blog/pico2pico/hero.png
mode: light
tags: 
 - raspberry_pi
 - pico
 - bluetooth
groups:
 - raspberrypi
 - pico
 - micropython
 - electronics
videos:
 - l-K_0N1m5kQ
code:
 - https://www.github.com/kevinmcaleer/pico_two_way_bluetooth
---

`Bluetooth Low Energy` (BLE) is a powerful technology that enables devices to communicate wirelessly over short distances. In this tutorial, we’ll explore how to use BLE to establish communication between two Raspberry Pi Pico W devices using MicroPython. One device will act as a **Central** (client), and the other as a **Peripheral** (server). We’ll send messages from the central to the peripheral, which will respond back, demonstrating a simple yet effective communication system.

## Why BLE and Raspberry Pi Pico W?

The Raspberry Pi Pico W is a microcontroller that comes with built-in Wi-Fi and BLE capabilities, making it perfect for IoT (Internet of Things) projects. BLE is especially suited for low-power, low-data-rate communications, ideal for connecting small devices like the Pico W.

---

## Project Overview

In this project, we’ll:

1. Set up one Raspberry Pi Pico W as a **Peripheral** (server).
2. Set up another Raspberry Pi Pico W as a **Central** (client).
3. Enable the central to send messages to the peripheral.
4. Have the peripheral respond back to the central.

We’ll use MicroPython, a lightweight version of Python designed to run on microcontrollers, to program our devices.

---

## Prerequisites

Before we dive in, make sure you have:

- Two Raspberry Pi Pico W devices.
- MicroPython installed on both devices.
- Basic understanding of Python and asynchronous programming.
- Thonny or another IDE set up for programming the Pico W.

---

## Understanding the Roles: Central vs. Peripheral

In BLE communication:

- **Peripheral (Server):** This device advertises itself and offers services that other devices can interact with. It waits for a central device to connect and can send data to the central.
- **Central (Client):** This device scans for peripherals, connects to them, and interacts with their services by reading, writing, or subscribing to notifications.

### UUIDs: The Identifiers

In BLE, **UUIDs (Universal Unique Identifiers)** are used to uniquely identify services and characteristics. We define these at the start of our program:

```python
_SERVICE_UUID = bluetooth.UUID(0x1848)
_CHARACTERISTIC_UUID = bluetooth.UUID(0x2A6E)
```

These UUIDs ensure that the central and peripheral are communicating over the correct channels.

---

## The Code Breakdown

Let’s walk through the key parts of the code that enable this BLE communication.

### 1. Setting Up the Peripheral (Server)

The peripheral is responsible for advertising its presence and providing a characteristic that the central can interact with.

```python
async def run_peripheral_mode():
    # Set up the Bluetooth service and characteristic
    ble_service = aioble.Service(BLE_SVC_UUID)
    characteristic = aioble.Characteristic(
        ble_service,
        BLE_CHARACTERISTIC_UUID,
        read=True,
        notify=True,
        write=True,
    )
    aioble.register_services(ble_service)

    print(f"{BLE_NAME} starting to advertise")

    while True:
        async with await aioble.advertise(
            BLE_ADVERTISING_INTERVAL,
            name=BLE_NAME,
            services=[BLE_SVC_UUID],
            appearance=BLE_APPEARANCE) as connection:
            print(f"{BLE_NAME} connected to another device: {connection.device}")

            tasks = [
                asyncio.create_task(send_data_task(connection, characteristic)),
            ]
            await asyncio.gather(*tasks)
            print(f"{IAM} disconnected")
            break
```

---

#### What’s Happening Here?

- **Service and Characteristic:** We create a service with a characteristic that the central can read from, write to, and receive notifications from.
- **Advertising:** The peripheral starts advertising itself, waiting for the central to connect.
- **Handling Connections:** Once connected, it sets up tasks to handle sending and receiving data.

---

### 2. Setting Up the Central (Client)

The central scans for the peripheral, connects to it, and sends data.

```python
async def run_central_mode():
    # Start scanning for a device with the matching service UUID
    while True:
        device = await ble_scan()

        if device is None:
            continue
        print(f"device is: {device}, name is {device.name()}")

        try:
            print(f"Connecting to {device.name()}")
            connection = await device.device.connect()

        except asyncio.TimeoutError:
            print("Timeout during connection")
            continue

        print(f"{IAM} connected to {connection}")

        # Discover services
        async with connection:
            try:
                service = await connection.service(BLE_SVC_UUID)
                characteristic = await service.characteristic(BLE_CHARACTERISTIC_UUID)
            except (asyncio.TimeoutError, AttributeError):
                print("Timed out discovering services/characteristics")
                continue
            except Exception as e:
                print(f"Error discovering services {e}")
                await connection.disconnect()
                continue

            tasks = [
                asyncio.create_task(receive_data_task(characteristic)),
            ]
            await asyncio.gather(*tasks)

            await connection.disconnected()
            print(f"{BLE_NAME} disconnected from {device.name()}")
            break
```

---

#### What’s Happening?

- **Scanning:** The central scans for peripherals advertising the specific service UUID.
- **Connecting:** Once it finds the peripheral, it connects and discovers the available services and characteristics.
- **Communication:** The central then listens for data from the peripheral and can send responses back.

---

### 3. Sending and Receiving Data

#### Sending Data

The central sends messages to the peripheral:

```python
async def send_data_task(connection, characteristic):
    global message_count
    while True:
        message = f"{MESSAGE} {message_count}"
        message_count += 1
        print(f"sending {message}")

        try:
            msg = encode_message(message)
            characteristic.write(msg)

            await asyncio.sleep(0.5)
            response = decode_message(characteristic.read())

            print(f"{IAM} sent: {message}, response {response}")
        except Exception as e:
            print(f"writing error {e}")
            continue

        await asyncio.sleep(0.5)
```

---

#### Receiving Data

The peripheral receives messages and can respond:

```python
async def receive_data_task(characteristic):
    global message_count
    while True:
        try:
            data = await characteristic.read()

            if data:
                print(f"{IAM} received: {decode_message(data)}, count: {message_count}")
                await characteristic.write(encode_message("Got it"))
                await asyncio.sleep(0.5)

            message_count += 1

        except asyncio.TimeoutError:
            print("Timeout waiting for data in {ble_name}.")
            break
        except Exception as e:
            print(f"Error receiving data: {e}")
            break
```

---

### 4. Encoding and Decoding Messages

BLE communication requires data to be sent in a specific format (bytes). We use two simple functions to encode and decode messages:

```python
def encode_message(message):
    return message.encode('utf-8')

def decode_message(message):
    return message.decode('utf-8')
```

---

### Running the Code

1. **Peripheral Device:** Set `IAM = "Peripheral"` and run the code on one Pico W.
2. **Central Device:** Set `IAM = "Central"` and run the code on the other Pico W.

The devices will establish a connection, and you’ll see messages being sent from the central to the peripheral and responses coming back.

---

## Conclusion

In this tutorial, we explored how to use MicroPython and BLE to enable two Raspberry Pi Pico W devices to communicate wirelessly. We set up one device as a central and the other as a peripheral, enabling a two-way communication system. This foundation can be expanded into more complex IoT applications, allowing devices to interact in real-time over BLE.

Whether you're building a simple remote control or a complex sensor network, understanding how to leverage BLE in your projects opens up a world of possibilities. Happy coding!

---
