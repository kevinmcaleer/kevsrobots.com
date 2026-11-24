---
title: PyPi and MIP
description: Install libraries onto your Pico with mip, from micropython-lib or GitHub, and understand why you cannot just pip install from PyPI
layout: lesson
type: page
cover: /learn/intermediate_micropython/assets/banners/09_pypi.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to install a library onto your Pico with `mip`, from the REPL or from your computer, and explain why PyPI packages usually do not work on a microcontroller.

## Do not write what you can install

Someone has already written a driver for your display, a client for MQTT and a Bluetooth library. Installing them saves you days. On a computer you would use `pip` and the Python Package Index, **PyPI**. A Pico is different.

### Why not PyPI?

PyPI hosts hundreds of thousands of packages written for desktop Python. Most assume a big standard library, plenty of RAM and an operating system with files and threads. A Pico has none of that, and `pip` itself does not run on it. A package that is pure Python and small might work if you copy the files by hand, but you cannot count on it.

### Meet `mip`

MicroPython has its own installer, **`mip`**, built to download libraries onto the board. By default it fetches from **micropython-lib**, the official collection of MicroPython-friendly libraries, and it can also pull files straight from GitHub, GitLab and Codeberg.

```mermaid
flowchart LR
    A["Your Pico"] -->|"mip.install(...)"| B{"Where from?"}
    B --> C["micropython-lib (default)"]
    B --> D["github:org/repo/file.py"]
    C --> E["/lib on the board"]
    D --> E
    E --> F["import it in your program"]
```

---

## Install from the Pico

`mip` downloads over Wi-Fi, so this route needs a board with Wi-Fi, such as the Pico W or Pico 2 W. Save and run this on the board, with your own network details:

```python
import network
import mip
from time import sleep

SSID = "your-network-name"
PASSWORD = "your-password"

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(SSID, PASSWORD)

for _ in range(20):
    if wlan.isconnected():
        break
    sleep(1)

if wlan.isconnected():
    print("Connected:", wlan.ifconfig()[0])
    mip.install("umqtt.simple")
else:
    print("Could not connect to Wi-Fi")
```

On success, `mip` prints which files it fetched and where it put them. Libraries land in the `/lib` folder, which MicroPython already searches, so afterwards you can simply write:

```python
from umqtt.simple import MQTTClient
```
{: .no-snakie }

Other useful names in micropython-lib include `aioble` (Bluetooth Low Energy) and `requests` (HTTP). The full catalogue is browsable at [github.com/micropython/micropython-lib](https://github.com/micropython/micropython-lib).

> **Careful with passwords.** Do not leave your real Wi-Fi password in code you share, post online or commit to GitHub.

### Options

- `mip.install("name", version="1.2")` installs a specific version.
- `mip.install("name", target="/lib")` chooses the destination folder.
- `mip.install("name", mpy=False)` fetches readable `.py` source instead of compiled `.mpy` files, handy when you want to read or edit a library.
- `mip.install("github:org/repo/file.py")` installs from a GitHub repository.

---

## Install from your computer with mpremote

You do not need Wi-Fi on the board for this one, as your computer does the downloading. If you have `mpremote` (next lesson), run:

```bash
mpremote mip install umqtt.simple
```

The same command accepts GitHub links, for example `mpremote mip install github:org/repo`, where `org/repo` is the repository you want. This is the best route for a Pico without Wi-Fi.

---

## A note on `urequests`

Older tutorials (including earlier versions of this course) say to install `urequests`. That name is out of date: HTTP requests now live in a module called **`requests`**, which is built into current Wi-Fi firmware and is also in micropython-lib. Try `import requests` first, and only install it if you get an `ImportError`.

> **Teachers**: Installing libraries needs a network. Many school Wi-Fi networks block devices that cannot sign in through a web page, which includes the Pico. Use `mpremote mip install` from a laptop on the school network instead, or download the library once and copy the file to each board. A phone hotspot is another option, and not all schools allow them.

---

## Try it Yourself

1. Run `mpremote devs` to list connected boards, then `mpremote mip install` any small library from micropython-lib and look in `/lib` using the Thonny Files pane.
2. Install a library with `mpy=False` and open the `.py` file to read how it works.
3. Look through micropython-lib on GitHub and note down two libraries that would help a robot project.
4. Challenge: find a library on GitHub with a single `.py` driver file and install it with `github:` syntax.

---

## Common Issues

- **Problem**: `OSError` or `Could not connect` when calling `mip.install`.
- **Solution**: Confirm `wlan.isconnected()` is `True` first, and check you are on a 2.4 GHz network.
- **Why**: `mip` downloads from the internet, so the board must be online. The Pico's Wi-Fi works on the 2.4 GHz band.

- **Problem**: `mip` says the package was not found.
- **Solution**: Check the exact name in micropython-lib, such as `umqtt.simple` rather than `umqtt`.
- **Why**: Package names must match what micropython-lib publishes.

- **Problem**: `import mip` fails with `ImportError`.
- **Solution**: Use `mpremote mip install ...` from your computer, or update your MicroPython firmware.
- **Why**: `mip` is only present on reasonably recent firmware.

---

## Check your understanding

{% quiz %}
? Why can't you simply `pip install` from PyPI onto a Pico?
- [ ] PyPI blocks microcontrollers
- [x] Most PyPI packages need more standard library and RAM than MicroPython offers, and `pip` does not run on the board
- [ ] Pico boards cannot store files
> PyPI packages are written for desktop Python.

? Where does `mip` put installed libraries by default?
- [x] In the `/lib` folder on the board
- [ ] On your computer's Desktop
- [ ] In RAM only
> `/lib` is already on the import search path.

? Which route works for a Pico with no Wi-Fi?
- [ ] `mip.install` run on the Pico
- [x] `mpremote mip install` run from your computer
- [ ] None of them
> The computer does the downloading and sends files over USB.
{% endquiz %}
