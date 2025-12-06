---
title: Setting up Your Development Environment
description: >-
    Set up Jupyter Lab and create a Python virtual environment for training custom object detection models with PyTorch
type: page
layout: lesson
---

## Overview

To build a custom object detection model, you'll need a proper development environment. In this lesson, you'll set up Jupyter Lab, which provides an interactive workspace perfect for machine learning experimentation. You'll also create an isolated Python environment to keep your project dependencies organized.

---

## Why Jupyter Lab?

Jupyter Lab provides an interactive environment perfect for machine learning work:

- **Iterative development** - Test code snippets quickly without running entire scripts
- **Visualization** - See your dataset, training progress, and results inline
- **Documentation** - Mix code, notes, and images in one place
- **Reproducibility** - Share notebooks with your team or community
- **Debugging** - Inspect variables and outputs at each step

Professional ML engineers use Jupyter for prototyping before deploying to production. It's the industry standard for exploratory data analysis and model development.

---

## Creating a Virtual Environment

A virtual environment keeps your project dependencies separate from your system Python installation. This prevents version conflicts and makes your project portable.

```bash
# Create a new virtual environment named 'venv'
python -m venv venv
```

**What this does:**
- Creates a new directory called `venv` containing a fresh Python installation
- Isolates your project dependencies from other Python projects
- Allows you to use specific package versions without affecting other projects

---

## Activating the Virtual Environment

Activate your virtual environment before installing packages:

**On macOS/Linux:**
```bash
source venv/bin/activate
```

**On Windows:**
```bash
venv\Scripts\activate
```

**You'll know it worked when:**
- Your terminal prompt shows `(venv)` at the beginning
- Running `which python` shows the path to your venv Python

---

## Install Required Packages

Now install Jupyter Lab and supporting tools:

```bash
pip install jupyterlab ipykernel label-studio
```

**What each package does:**
- `jupyterlab` - Interactive development environment
- `ipykernel` - Allows Jupyter to run Python code
- `label-studio` - Tool for annotating images (we'll use this later)

**Installation time:** About 2-3 minutes depending on your internet speed.

---

## Add the Virtual Environment to Jupyter Lab

Register your virtual environment as a Jupyter kernel:

```bash
python -m ipykernel install --user --name=venv --display-name "Python (venv)"
```

**What this does:**
- Makes your virtual environment available in Jupyter Lab
- Allows you to select "Python (venv)" as your notebook kernel
- Ensures notebooks use the correct Python environment with your installed packages

---

## Running Jupyter Lab

Start Jupyter Lab with this command:

```bash
jupyter lab --ip=0.0.0.0 --port=8888 --no-browser
```

**Command options explained:**
- `--ip=0.0.0.0` - Allows access from any network interface (useful for remote access)
- `--port=8888` - Runs on port 8888 (default Jupyter port)
- `--no-browser` - Doesn't auto-open a browser (you'll open it manually)

**Expected output:**
```
[I 2024-10-16 10:30:00.000 ServerApp] Jupyter Server is running at:
[I 2024-10-16 10:30:00.000 ServerApp] http://localhost:8888/lab?token=abc123...
```

Copy the URL with the token and paste it into your browser.

---

## Verify Your Setup

Let's confirm everything is installed correctly. Create a new notebook in Jupyter Lab and run this code:

```python
# test_setup.py
import sys
import jupyterlab
import ipykernel

print("✓ Python version:", sys.version)
print("✓ Jupyter Lab version:", jupyterlab.__version__)
print("✓ IPyKernel version:", ipykernel.__version__)
print("\nSetup successful! You're ready to go.")
```

**Expected output:**
```
✓ Python version: 3.10.12 (main, Jun 11 2023, 05:26:28)
✓ Jupyter Lab version: 4.0.0
✓ IPyKernel version: 6.25.0

Setup successful! You're ready to go.
```

---

## Try It Yourself

1. **Create a new notebook** - In Jupyter Lab, click the "+" button and select "Python (venv)"
2. **Test basic Python** - Type `print("Hello, AI Camera!")` in a cell and press Shift+Enter
3. **Install matplotlib** - In a new cell, run `!pip install matplotlib`

**Challenge:** Create a simple plot to verify matplotlib is working:

```python
import matplotlib.pyplot as plt

plt.figure(figsize=(8, 4))
plt.plot([1, 2, 3, 4], [1, 4, 2, 3])
plt.title('Test Plot')
plt.xlabel('X axis')
plt.ylabel('Y axis')
plt.show()
```

You should see a line graph appear below the cell!

---

## Common Issues

**Problem**: "python: command not found"
**Solution**: Install Python from [python.org](https://python.org) or use your package manager (`brew install python3` on Mac, `sudo apt install python3` on Linux)
**Why**: Python needs to be installed on your system first

**Problem**: "pip: command not found"
**Solution**: Try `python -m pip` instead of `pip`, or reinstall Python with pip included
**Why**: pip might not be in your PATH, or wasn't installed with Python

**Problem**: Virtual environment won't activate on Windows
**Solution**: Run `Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser` in PowerShell
**Why**: Windows security policy blocks script execution by default

**Problem**: Can't access Jupyter Lab from browser
**Solution**: Check firewall settings, or use `localhost:8888` instead of `0.0.0.0:8888` in the URL
**Why**: Firewall may be blocking the port or network configuration issue

**Problem**: "Kernel dead" or "Kernel error" in Jupyter
**Solution**: Restart the kernel (Kernel → Restart Kernel), or recreate the ipykernel installation
**Why**: Package conflict or incomplete installation

---

## Summary

You've learned:
- Why Jupyter Lab is essential for ML development
- How to create isolated Python environments with venv
- Installing and configuring Jupyter Lab
- Verifying your setup with test code
- Troubleshooting common installation issues

Your development environment is now ready! In the next lesson, you'll learn how to prepare a custom dataset for training your object detection model.

---

## Next Lesson

In the next lesson, you'll learn how to collect and annotate images to create a custom object detection dataset.

---
