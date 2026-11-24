---
title: Your First Program in C
description: Create your own Pico project with a CMakeLists.txt, print Hello over USB serial, and read it on your computer.
layout: lesson
type: page
cover: assets/7.jpg
date_updated: 2026-11-24
---

![Cover]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to create a Pico project from scratch, build it, and read its `printf` output over USB.

Last lesson you built someone else's program. Now let's build our own. We'll make a robot that says hello, and keeps counting so you can see it is alive. The tools are the same as before, so if lesson 6 worked, this will too.

```mermaid
flowchart LR
    A["hello.c"] --> B["CMakeLists.txt<br/>+ pico_sdk_import.cmake"]
    B --> C["cmake + make"]
    C --> D["hello.uf2"]
    D --> E["Pico runs it<br/>printf goes out over USB"]
    E --> F["Serial terminal<br/>on your computer"]
```

---

## Step 1: Create the project folder

```bash
mkdir -p ~/pico/my-project
cd ~/pico/my-project
cp $PICO_SDK_PATH/external/pico_sdk_import.cmake .
```

That last line copies the small helper file from the SDK that lets CMake find it. Every Pico project needs it.

## Step 2: The program

Create `hello.c`:

```c
#include <stdio.h>
#include "pico/stdlib.h"

int main() {
    stdio_init_all();
    sleep_ms(2000);

    int count = 0;
    while (true) {
        printf("Hello from the Pico! Count: %d\n", count);
        count++;
        sleep_ms(1000);
    }
}
```

What's new compared with desktop C:

- `#include "pico/stdlib.h"` brings in the SDK's standard library, including `sleep_ms` and the GPIO functions. SDK headers use quotes; system headers like `stdio.h` use angle brackets.
- `stdio_init_all()` switches on the Pico's standard input and output so `printf` has somewhere to go.
- `sleep_ms(2000)` waits two seconds before the first message. Your computer needs a moment to notice the board and open the serial port after it starts, and anything printed before that is lost.
- `while (true)` never ends. A Pico program has no operating system to return to, so `main` keeps going forever.

## Step 3: The CMakeLists.txt

Create `CMakeLists.txt` in the same folder:

```cmake
cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(my_project C CXX ASM)

pico_sdk_init()

add_executable(hello hello.c)

target_link_libraries(hello pico_stdlib)

pico_enable_stdio_usb(hello 1)
pico_enable_stdio_uart(hello 0)

pico_add_extra_outputs(hello)
```

Line by line:

- `include(pico_sdk_import.cmake)` must come before `project(...)`.
- `pico_sdk_init()` sets up the SDK.
- `add_executable(hello hello.c)` makes a program called `hello` from our source file.
- `target_link_libraries(hello pico_stdlib)` links in the SDK library.
- `pico_enable_stdio_usb(hello 1)` sends `printf` output over USB. `pico_enable_stdio_uart(hello 0)` turns off the other route, the serial pins. Without these lines, the output goes to the UART pins and nothing shows on your computer.
- `pico_add_extra_outputs(hello)` is what produces the `.uf2` file.

## Step 4: Build

```bash
mkdir build
cd build
cmake ..
make
```

For a Pico W add `-DPICO_BOARD=pico_w` to the `cmake` command, and for a Pico 2 add `-DPICO_BOARD=pico2`. You will find `hello.uf2` in the `build` folder. Copy it onto the BOOTSEL drive just as in lesson 6.

## Step 5: Read the output

Linux, with `screen`:

```bash
screen /dev/ttyACM0 115200
```

The device name may differ: check `ls /dev/ttyACM*`. On macOS it looks like `/dev/tty.usbmodem...`, and on Windows the board appears as a COM port that you can open with a terminal program such as PuTTY or Tera Term. To leave `screen`, press Ctrl+A, then K, then Y.

You should see:

```text
Hello from the Pico! Count: 0
Hello from the Pico! Count: 1
Hello from the Pico! Count: 2
```

> **Teachers**: the Pico shows up as a serial device. On Linux learners may need permission to open `/dev/ttyACM0` (on many systems that means being in the `dialout` group), which needs an administrator. Check this before the lesson. If several Picos are plugged into one machine each gets its own device name, so label them.

---

## Try it Yourself

1. Change the message to include your name.
2. Make it count in tens instead of ones.
3. Change `sleep_ms(1000)` to `sleep_ms(250)` and rebuild.
4. Challenge: use `if (count % 5 == 0)` to print a special line every fifth message.

## Common Issues

- **Problem**: The serial terminal opens but shows nothing.
- **Solution**: Check `pico_enable_stdio_usb(hello 1)` is in `CMakeLists.txt`, rebuild, and re-copy the new `.uf2`.
- **Why**: Without it `printf` writes to the UART pins, not USB.

- **Problem**: The first line or two are missing.
- **Solution**: Keep the `sleep_ms(2000)` after `stdio_init_all()`.
- **Why**: The computer needs time to connect to the serial port, and early output is lost.

- **Problem**: `screen: Cannot open line '/dev/ttyACM0': Permission denied` or the device is not there.
- **Solution**: Check `ls /dev/ttyACM*`, and ask an admin to add you to the `dialout` group if needed. Make sure the Pico is not still in BOOTSEL mode.
- **Why**: Serial ports are protected, and in BOOTSEL mode the board is a drive instead of a serial device.

- **Problem**: CMake cannot find `pico_sdk_import.cmake`.
- **Solution**: Copy it into the project folder again, next to `CMakeLists.txt`.
- **Why**: `include()` looks for it relative to the project.

## Check your understanding

{% quiz %}
? Which CMake line makes the `.uf2` file?
- [ ] `add_executable`
- [x] `pico_add_extra_outputs`
- [ ] `pico_sdk_init`
> `pico_add_extra_outputs` produces the extra output formats, including `.uf2`.

? Why is there a `sleep_ms(2000)` after `stdio_init_all()`?
- [ ] To warm up the chip
- [x] So your computer can open the serial port before the first message
- [ ] Because `printf` is slow
> Output sent before the host connects is lost.

? What does `pico_enable_stdio_usb(hello 1)` do?
- [x] Sends `printf` output over USB
- [ ] Powers the board from USB
- [ ] Starts BOOTSEL mode
> It enables USB serial output for `printf`.
{% endquiz %}
