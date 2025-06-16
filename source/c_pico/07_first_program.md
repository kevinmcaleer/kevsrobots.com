---
title: Your First Program in C
description: Write and upload your very first C program for the Raspberry Pi Pico – a custom "Hello, world!" using the onboard serial console.
layout: lesson
type: page
cover: assets/7.jpg
date_updated: 2025-06-15
---

![Cover]({{page.cover}}){:class="cover"}

---

You’ve set up your toolchain — now let’s write and upload your **very first C program** to the Raspberry Pi Pico!

Instead of just blinking an LED, we’ll start with a classic: **"Hello, world!"**  
But this time, it will run on real hardware.

---

## Step 1: Create a New Project

Let’s make a folder for your own code:

```bash
mkdir -p ~/pico/my-project
cd ~/pico/my-project
````

Create these files:

### `CMakeLists.txt`

The CMake file tells the build system how to compile your code. Create a file named `CMakeLists.txt` with the following content:

```cmake
cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(my_project)

pico_sdk_init()

add_executable(hello
    hello.c
)

target_link_libraries(hello pico_stdlib)

pico_add_extra_outputs(hello)
```

### `hello.c`

Create a file named `hello.c` - this is the main program. Add the following content to it:

```c
#include <stdio.h>
#include "pico/stdlib.h"

int main() {
    stdio_init_all();

    while (true) {
        printf("Hello, world from the Pico!\n");
        sleep_ms(1000);
    }
}
```

### `pico_sdk_import.cmake`

Copy this file from `~/pico/pico-examples`:

```bash
cp ~/pico/pico-examples/pico_sdk_import.cmake .
```

---

## Step 2: Build the Program

```bash
mkdir build
cd build
cmake ..
make
```

You should now have `hello.uf2` in the build folder.

---

## Step 3: Upload to Your Pico

1. Hold down **BOOTSEL**
2. Plug in your Pico
3. Drag and drop `hello.uf2` onto the **RPI-RP2** USB drive

The Pico will reboot and start running your program.

---

## Step 4: View Output Over USB Serial

Use `minicom`, `screen`, or a serial monitor to view the output:

### On Linux/macOS:

```bash
screen /dev/ttyACM0 115200
```

> You may need to install `screen`:
>
> ```bash
> sudo apt install screen
> ```

You should see:

```bash
Hello, world from the Pico!
Hello, world from the Pico!
...
```

---

## Summary

You just:

* Created your own C project from scratch
* Compiled it using CMake and the Pico SDK
* Uploaded the binary to your Pico
* Saw real-time output using USB serial

🎉 Congratulations — you're now officially writing C code on the Raspberry Pi Pico!

---

Next up: [GPIO Basics](08_gpio_basics), where we’ll learn how to interact with the outside world by controlling physical pins.

---
