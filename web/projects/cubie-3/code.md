---
title: "Cubie-3 Code"
description: >-
    An open-source 3d printable cube robot you can build yourself
excerpt: >-
    
layout: multipage
date: 2026-01-07
author: Kevin McAleer
difficulty: beginner
cover: /projects/cubie-3/assets/img/cover.jpg
hero:  /projects/cubie-3/assets/img/hero.png
mode: light
tags:
 - robots
 - cubie-3
groups:
 - robots
videos:
 - K-bFD4RvypU
navigation:
 - name: "Overview"
   link: index
 - name: "Bill of Materials"
   link: bom
 - name: "Circuit Diagram"
   link: circuit
 - name: "Wiring"
   link: wiring
 - name: "Assembly"
   link: assembly
 - name: "Code"
   link: code
 - name: "Downloadable STL files"
   link: stl
next: stl
previous: assembly
---

The full version controlled code for this project is available at [Cubie-3 GitHub Respository](https://www.github.com/kevinmcaleer/cubie-3).

> ## Note
>
> This code requires the Sunfounder Fusion HAT+ Python library. You can find installation instructions in the [Sunfounder Fusion HAT+ - What Can't This Thing Do?](https://www.youtube.com/watch?v=K-bFD4RvypU) video.

---

Example code to drive the motors:

```python
from cubie_3 import Cubie3
import asyncio

async def main():
    robot = Cubie3()

    # Move forward at 30% speed
    await robot.forward(0.3)
    await asyncio.sleep(1)

    # Strafe right
    await robot.strafe_right(0.5)
    await asyncio.sleep(0.5)

    # Stop all motors
    await robot.stop()

asyncio.run(main())
```

---

## Voice Recognition with Vosk

Cubie-3 uses [Vosk](https://alphacephei.com/vosk/) for offline speech-to-text recognition. This means your robot can respond to voice commands without needing an internet connection - all processing happens locally on the Raspberry Pi.

### What is Vosk?

Vosk is a lightweight, offline-capable speech recognition engine that:

- Works without an internet connection
- Supports 20+ languages
- Runs on resource-constrained devices like Raspberry Pi
- Has small language models (as small as 50MB)

### How It Works in Cubie-3

The voice recognition system uses the Sunfounder Fusion HAT+ library which wraps Vosk functionality. When you call the `listen()` method:

1. **Audio capture** - The USB microphone captures audio continuously
2. **Real-time processing** - Vosk processes audio in a stream, providing partial results as you speak
3. **Command matching** - When a final result is recognized, Cubie-3 checks if it matches a known command
4. **Action execution** - Matching commands trigger the corresponding robot movement

---

## Setting Up Voice Recognition

### 1. Hardware Requirements

- USB microphone (essential for clear audio input)
- Sunfounder Fusion HAT+ installed on your Raspberry Pi

### 2. Configure Your Microphone

First, identify your USB microphone:

```bash
arecord -l
```

You'll see output like:

```
card 1: Device [USB Audio Device], device 0: USB Audio [USB Audio]
```

Test recording (replace `1,0` with your card and device numbers):

```bash
arecord -D plughw:1,0 -f S16_LE -r 16000 -d 3 test.wav
aplay test.wav
```

If the recording is too quiet or muted, adjust levels:

```bash
alsamixer
```

- Press `F6` to select your USB microphone
- Find the Mic or Capture channel
- Ensure it shows `[OO]` (unmuted) - press `M` to toggle
- Use arrow keys to adjust volume

### 3. Install Dependencies

The Fusion HAT+ library handles Vosk installation, but if you need to install manually:

```bash
sudo apt install portaudio19-dev python3-pyaudio
pip3 install vosk
```

The language model downloads automatically on first use.

---

## Voice-Controlled Demo

Here's the complete voice control example from the Cubie-3 repository:

```python
from cubie_3 import Cubie3
import asyncio

async def main():
    cubie3 = Cubie3()

    # Announce startup
    cubie3.tts.say("Starting movement demo.")

    # Start listening for voice commands
    await cubie3.listen()

if __name__ == "__main__":
    asyncio.run(main())
```

---

## Recognized Voice Commands

Cubie-3 responds to these voice commands:

| Command | Action |
|---------|--------|
| "forward" | Move forward |
| "backward" | Move backward |
| "strafe left" | Move sideways to the left |
| "strafe right" | Move sideways to the right |
| "rotate left" | Spin counter-clockwise |
| "rotate right" | Spin clockwise |
| "stop" | Stop all motors |
| "goodbye" | End the listening session |
{: .table .table-single }

---

## How the Listen Method Works

The `listen()` method in the Cubie3 class implements continuous speech recognition:

```python
# Simplified example of how listen() works internally
from fusion_hat.stt import Vosk as STT

stt = STT(language="en-us")

while True:
    print("Listening...")
    for result in stt.listen(stream=True):
        if result["done"]:
            # Final recognized text
            command = result['final'].lower()
            if "forward" in command:
                await robot.forward(0.5)
            elif "stop" in command:
                await robot.stop()
            # ... more commands
        else:
            # Partial result (updates as you speak)
            print(f"Heard: {result['partial']}", end="\r")
```

---

## Text-to-Speech (TTS)

Cubie-3 can also speak back to you using the `say()` method:

```python
from cubie_3 import Cubie3

robot = Cubie3()

# Robot speaks using Espeak TTS
robot.tts.say("Hello, I am Cubie-3!")
robot.tts.say("Ready for your commands.")
```

The TTS system uses Espeak, which is lightweight and built into Raspberry Pi OS. While the voice sounds robotic, it's highly configurable for volume, pitch, and speed.

---

## Tips for Better Voice Recognition

1. **Microphone position** - Keep the microphone 15-30 cm from your mouth
2. **Quiet environment** - Background noise reduces accuracy
3. **Clear speech** - Speak clearly and at a normal pace
4. **Pause between commands** - Give the system time to process

---

## Troubleshooting

**"No audio detected"**

- Check microphone connection with `arecord -l`
- Verify volume levels in `alsamixer`
- Test recording with `arecord` command

**"Commands not recognized"**

- Speak the exact command phrases
- Check you're using the correct language model
- Try speaking more slowly and clearly

**"Model download fails"**

- Check internet connection (only needed for initial download)
- Manually download from [Vosk Models](https://alphacephei.com/vosk/models)
- Use the small English model: `vosk-model-small-en-us-0.15`

---

## Supported Languages

Vosk supports many languages. Change the language when initializing:

```python
# For British English
stt = STT(language="en-gb")

# For German
stt = STT(language="de")

# For French
stt = STT(language="fr")
```

Available language codes include: `en-us`, `en-gb`, `de`, `fr`, `es`, `it`, `pt`, `ru`, `cn`, `ja`, `ko`, and many more.

---

## Further Resources

- [Vosk Official Documentation](https://alphacephei.com/vosk/)
- [Vosk GitHub Repository](https://github.com/alphacep/vosk-api)
- [Sunfounder Fusion HAT+ STT Documentation](https://docs.sunfounder.com/projects/fusion-hat/en/latest/ai_interaction/python_stt_vosk.html)
- [Vosk Language Models](https://alphacephei.com/vosk/models)

