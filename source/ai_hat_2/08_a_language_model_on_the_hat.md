---
title: A language model on the HAT
description: Install hailo-ollama and run a small language model on the AI HAT+ 2 from the terminal and from Python
layout: lesson
type: page
cover: /learn/ai_hat_2/assets/banners/08_a_language_model_on_the_hat.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Now the brain. The AI HAT+ 2 can run a language model, the kind of AI behind chatbots, without going near the internet once the model is downloaded. The Raspberry Pi docs describe three parts:

- the **Hailo Gen-AI Model Zoo**, the models
- the **Hailo Ollama server** (`hailo-ollama`), the backend with a REST API
- **Open WebUI**, an optional page in your browser (lesson 9)

## Which models?

At launch, the Raspberry Pi announcement lists these: DeepSeek-R1-Distill (1.5 billion parameters), Llama3.2 (1 billion), Qwen2.5-Coder (1.5 billion), Qwen2.5-Instruct (1.5 billion) and Qwen2 (1.5 billion). The announcement says larger models are expected soon after launch.

## Step 1: install the server

The docs say to download Hailo Model Zoo GenAI version 5.1.1 from:

```text
https://dev-public.hailo.ai/2025_12/Hailo10/hailo_gen_ai_model_zoo_5.1.1_arm64.deb
```

Then install it:

```bash
sudo dpkg -i hailo_gen_ai_model_zoo_5.1.1_arm64.deb
```

Check the docs page for the current version, as it may have moved on.

## Step 2: start the server and chat

Run the server in one terminal:

```bash
hailo-ollama
```

In a second terminal, list the models, then pull one and chat. This is the sequence from the docs, where `examplemodel:tag` is a model from the list, such as `qwen2:1.5b`:

```bash
curl --silent http://localhost:8000/hailo/v1/list

curl --silent http://localhost:8000/api/pull \
     -H 'Content-Type: application/json' \
     -d '{ "model": "examplemodel:tag", "stream" : true }'

curl --silent http://localhost:8000/api/chat \
     -H 'Content-Type: application/json' \
     -d '{"model": "examplemodel:tag", "messages": [{"role": "user", "content": "Translate to French: The cat is on the table."}]}'
```

The first download takes a while, as model files are big. The CNX Software review noted the model files it tried added up to about 11 GB, and a 32 GB microSD card reached 96% full, so keep an eye on your storage.

## Talking to it from Python

Because it is a normal web API, Python can talk to it with the standard library. This sends the same request as the last `curl` command and prints whatever the server returns. Change the model name to one you pulled.

```python
import json
import urllib.request

URL = "http://localhost:8000/api/chat"
MODEL = "qwen2:1.5b"   # use a name from the list command


def ask(question):
    payload = {
        "model": MODEL,
        "messages": [{"role": "user", "content": question}],
    }
    request = urllib.request.Request(
        URL,
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(request, timeout=120) as response:
        return response.read().decode("utf-8")


if __name__ == "__main__":
    try:
        print(ask("In one sentence, what should a robot do if it loses sight of its owner?"))
    except OSError as error:
        print("Could not reach hailo-ollama on port 8000:", error)
        print("Is it running in another terminal?")
```

The docs don't show the reply format, so the program prints it exactly as it arrives. Read it, and then decide how to pick out the text for your robot.

**Real robot example:** the language model can turn a messy sentence from a person into something your code can act on. For instance, you could ask it to answer with one word from `forward`, `stop` or `search`, and check the answer in the same way as the commands in lesson 7.

## What speed to expect

Be realistic. In the CNX Software review (January 2026), which tested a Raspberry Pi 5 2GB with the HAT, the results ranged from about 2.6 tokens per second for Llama 3.2 3B to about 8.1 for Qwen2.5 Coder 1.5B. A token is roughly a word or part of a word. These are one reviewer's numbers, not official ones, and your results will vary. The point is that a short answer takes a few seconds, not a split second.

## Try it Yourself

1. Run the `list` command and write down the model names you see.
2. Ask the model the same question with two different models, and compare the answers.
3. Challenge: change the question in the Python program so it asks the model to reply with a single word.

## Common Issues

**Problem**: `curl` says connection refused.
**Solution**: Start `hailo-ollama` in the first terminal and leave it running.
**Why**: The commands in the second terminal talk to that server.

**Problem**: The disk fills up.
**Solution**: Use a larger card, and remove models you don't use.
**Why**: Model files are large.

## Check your understanding

{% quiz %}
? Which program is the backend that serves language models on the AI HAT+ 2?
- [ ] `rpicam-hello`
- [x] `hailo-ollama`
- [ ] `hailortcli`
> `hailo-ollama` is the Hailo Ollama server. It has a REST API on port 8000.

? Why run `hailo-ollama` in one terminal and `curl` in another?
- [ ] `curl` only works on a second screen
- [x] The server has to stay running while the second terminal sends it requests
- [ ] It is faster
> The docs say to run `hailo-ollama` in one terminal and the other commands in a second.
{% endquiz %}
