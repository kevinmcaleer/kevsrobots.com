---
title: Setting up Jupyter Notebook
description: 
type: page
layout: lesson
---

To build a custom object detection model, you will need to set up some tools; for this tutorial we will use a Jupyter Notebook environment to run your code. In this lesson, you will learn how to install Jupyter Notebook and set up a virtual environment to run your code.

---

## Creating a Virtual Environment

First, we need to create a virtual environment to install the required packages for our project. We will use the `venv` module to create a virtual environment.

```bash
python -m venv venv
```

---

## Activating the Virtual Environment

Next, we need to activate the virtual environment to install the required packages.

```bash
source venv/bin/activate
```

---

## Install Jupyter Notebook

First we need to install `Jupyter Lab` software, `ipykernel` and `label-studio`, using pip:

```bash
pip install jupyterlab ipykernel label-studio
```

---

## Add the virtual environment to Jupyter Lab

Next, we need to add the virtual environment to Jupyter Lab so that we can run our code in the virtual environment.

```bash
python -m ipykernel install --user --name=venv --display-name "Python (venv)"
```

---

## Running Jupyter Notebook

To run Jupyter Lab, use the following command:

```bash
jupyter lab --ip=0.0.0.0 --port=8888 --no-browser
```

The command line options here are:

- `--ip=0.0.0.0`: This tells Jupyter Lab to listen on all network interfaces.
- `--port=8888`: This tells Jupyter Lab to listen on port 8888.
- `--no-browser`: This tells Jupyter Lab not to open a browser window.

---
