---
layout: pythonai
title: Part 1 - Creating the Assistant
subtitle: Learn how to build your own Jarvis-like AI assistant, in Python
thanks: true
description: Learn how to build your own Jarvis-like AI assistant, in Python
excerpt: Learn how to build your own Jarvis-like AI assistant, in Python
---

Welcome to the Build Your Own AI in Python series! We cover a lot of content in this series. It makes sense to start at the beginning as each part builds on the previous lesson and each also contains vital skills and knowledge used in future lessons too.

### Part 1 - Table of Contents

{:toc}
* toc

### Video on YouTube
I created a popular video series on YouTube, which goes over each of the steps below. The first video, `Creating the Assistant` is featured below.

{% include youtubeplayer.html id="Y5atyJbVsAU" %}

---

### Session Goals

In this session we will learn:

* Overview of this series
* How AI Assistants work
* What you'll need
* Adding AI Skills - what can we expect to achieve

[![Goals Diagram](/assets/img/pythonai/part1_002.png){:class="img-fluid w-25"}](/assets/img/pythonai/part1_002.png)

---

### Overview of this series
How we will create an AI Assistant

* We will use a Raspberry Pi Computer to run the code
* We will create code to listen to commands
* We will create code to say responses
* We will create a Web User Interface, so we can check what its doing and access it from our mobile devices
* We will add Skills, so that our AI will be able to connect to web services and APIs


[![How do AI Assistants Diagram](/assets/img/pythonai/part1_003.png){:class="img-fluid w-25"}](/assets/img/pythonai/part1_003.png)

---

### How do AI Assistants Work?
So how do AI Assistants actually work? Lets take a look at the logical flow...

[![How do AI Assistants Diagram](/assets/img/pythonai/part1_004.png){:class="img-fluid w-25"}](/assets/img/pythonai/part1_004.png)

#### Logic Flow
* Listen for a wake word
* If heard listen for a known command (use elastic search and fuzzy logic to derive command from many possible ways of saying it)
* Execute the known command
* Speak response

[![How do AI Assistants Diagram](/assets/img/pythonai/part1_005.png){:class="img-fluid w-25"}](/assets/img/pythonai/part1_005.png)

---

### A overview sketch of out AI Assistant
The Initial idea is:
* The core AI code is a loop, listening to speech, or taking actions from the Web UI
* We can add Skills as we go
  * Home Automation
  * Weather
  * News
  * Fun - Jokes

[![How do AI Assistants Diagram](/assets/img/pythonai/part1_007.png){:class="img-fluid w-25"}](/assets/img/pythonai/part1_007.png)

---

### AI Assistant Architecture
Our AI Assistant is made up of a number of parts:
* The Speech Engine
  * Speech Recognition
  * Speech Synthesis
* The Orchestration Engine
* The Web UI
* The Skills
  * News
  * Insults
  * Jokes
  * Calendar
  * Weather
  * Home Automation

[![How do AI Assistants Diagram](/assets/img/pythonai/part1_008.png){:class="img-fluid w-25"}](/assets/img/pythonai/part1_008.png)

---

### Architecture, the four pillars
We will therefore need to create each of the four pillars, and as far as possible ensure they have low coupling and high cohesion. Low coupling means each module/pillar should be self contained, other modules shouldn't rely on specifics of how they work internally. High Cohesion means that the code inside each module/pillar should work as effectively as possible, with each part doing only what it is supposed to do, and no more.

The four pillars are:

* Speech Engine
* Skills class
* Web UI
* Orchestration Engine

[![How do AI Assistants Diagram](/assets/img/pythonai/part1_009.png){:class="img-fluid w-25"}](/assets/img/pythonai/part1_009.png)

---

### Hardware and software toolkit
For this project we'll need a couple of things, some hardware to run our AI Assistant on, and some software tools to build it.

[![How do AI Assistants Diagram](/assets/img/pythonai/part1_012.png){:class="img-fluid w-25"}](/assets/img/pythonai/part1_012.png)

---

### Hardware
For hardware, you can run the Assistant on your own computer, *Mac*, *Windows PC* or *Linux PC*,

* **Raspberry Pi** - the faster the better, recommend a Raspberry Pi 4
* **USB Microphone** - to enable the Python to hear speech 
* **A Speaker** - need to be able to hear what is being said

[![How do AI Assistants Diagram](/assets/img/pythonai/part1_013.png){:class="img-fluid w-25"}](/assets/img/pythonai/part1_013.png)

---

### Software
For software, we will be using the following software applications:

* [Postman](https://www.postman.com) - for poking and prodding APIs
* [Bootstrap](https://www.bootstrap.com) - for building nice web interfaces
* [GitHub](https://www.github.com) - for storing our code securely
* [Mirosoft Visual Studio Code](https://code.visualstudio.com) - for writing our code
* [Flask](https://flask.palletsprojects.com) - for building nice APIs and websites in Python
* [Python](https://www.python.org) - The latest version of Python

[![How do AI Assistants Diagram](/assets/img/pythonai/part1_014.png){:class="img-fluid w-25"}](/assets/img/pythonai/part1_014.png)

---

### Skills

We want to add new skills over time as we develop them; lets create a skills framework:

* Skill have a name and description
* Skills have one or more actions
* Skills have a version
* Skills return a success or failure status
* Skills can have a web UI

[![How do AI Assistants Diagram](/assets/img/pythonai/part1_016.png){:class="img-fluid w-25"}](/assets/img/pythonai/part1_016.png)

---

### Speech engine

For the Raspberry Pi, and other Linux systems you will need to ensure that both `portaudio` and `espeak` are installed:

#### Install the Pre-requisites

``` bash
sudo apt-get install portaudio
sudo apt-get install espeak
```

#### Create the virtual environment:

``` bash
sudo python3 -m venv venv
source activate venv\bin\activate
```

To deactivate

``` bash
deactivate
```

#### Install the packages

Once you’ve activated your virtual environment
``` bash 
pip install pyaudio
pip install speechrecognition
pip install pyttsx3
```

Or

``` bash
pip install -r requirements.txt
```

---

### The code
The source code repository for this project is hosted at GitHub. Click this link: <https://github.com/kevinmcaleer/pythonai> to download the latest version of the code.

Please note that as the series progresses, code is changes, so you will be working from the latest version of the code.