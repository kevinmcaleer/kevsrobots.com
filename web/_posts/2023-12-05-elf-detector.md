---
title: Elf detector
description: A Raspberry Pi powered robot that can find elves!
layout: project
date: 2023-12-09
cover: /assets/img/blog/elf/elf.jpg
excerpt:
author: Kevin McAleer
difficulty: Intermediate
groups:
    - robot
    - micropython
    - seasonal
    - raspberrypi
tags:
    - robots
    - ai
    - machine learning
    - opencv
    - viam
---

[`Viam`](https://app.viam.com) has a really easy to use machine learning training system that can be used to train a model to recognise objects. I thought it would be fun to train a model to recognise Elves, and then use it on a robot that could find them.

---

## The process of training a model

The process of training a model to recognise objects contains following steps:

[![Model training process](/assets/img/blog/elf/process.png){:class="img-fluid w-100 shadow-lg"}](/assets/img/blog/elf/process.png)

1. [**Data Collection**](#capturing-images) -  Capture images of the object you want to train the model to recognise
1. [**Labeling**](#prepare-the-images-for-training) - Data Preparation Prepare the images for training
1. [**Model Selection**](#deploying-the-model) - Choose the type of model you want to train
1. [**Train Model**](#train-the-model-from-the-dataset) - Train the model using the images you captured
1. [**Model Evaluation**](#evaluating-the-model) - Test the model to see how well it performs
1. [**Fine Tuning**](#fine-tuning-the-model) - Improve the model by adding more images
1. [**Model Deployment**](#deploying-the-model) - Deploy the model to a robot

---

## Setting up Viam

First we need to setup Viam on our Raspberry Pi. Viam is a cloud based software platform that allows you to easily create and manage robots. It also has a really easy to use machine learning training system that can be used to train a model to recognise objects.

### The robot

I'm using an off the shelf `Rover` robot from Viam, which is powered by the Raspberry Pi 4. The robot has a webcam connected and this can be used to capture images our our elf from different positions to train our model. It then use the camera later to detect elves within the robots field of view.

![Rover robot](/assets/img/blog/elf/rover.png){:class="img-fluid w-100"}

---

### Creating a robot in Viam

To create a robot in Viam, you need to log into the Viam software at [app.viam.com](https://app.viam.com). Once you have logged in, you can create a new robot by adding the name of the new robot and clicking on the `Add Robot` button in the top left corner of the screen.

![Viam create robot](/assets/img/blog/elf/add_robot.png){:class="img-fluid w-100 shadow-lg"}

---

### Installing Viam on the Raspberry Pi

To install the `viam-server` on the Raspberry Pi, follow 3 the onscreen instructions below. This assumes you've some knowledge about how to setup and use an SSH session to connect to your Raspberry Pi. Luckily, Viam has a really easy to follow guide to help you do this, available [here](https://docs.viam.com/get-started/installation/prepare/rpi-setup/).

1. **Download the Viam app** - copy and paste the command into an ssh session connected to your raspberry pi

1. **Download and install the viam-server service** - again copy and paste the command into an ssh session connected to your raspberry pi.

1. **Check for connectivity** - Finally, Viam will report back that it has detected that the viam-server is now running on the Raspberry Pi and will present a button to configure the robot.

![Installing Viam Screenshot](/assets/img/blog/elf/install.png){:class="img-fluid w-100 shadow-lg"}

---

### Add a Pi board to the robot

Next we need to add a Pi board to our robot. This will allow us to control the robot from the Viam software. To do this:

1. Click on the `Config` tab on the robot we want to add the Pi board to. This will open up the configuration screen for the robot
1. Click the `+ Create Component` button to add a new component to the robot
1. Select `board`, and then `Pi` from the list of components
1. Give the Pi board a name; be sure to avoid spaces and special characters.
1. Click the `Save config` button

---

### Configuring the robot

Next we need to add a camera to our robot so that it can see. The easiest way to do this on Viam and Raspberry Pi is to plug in a cheap USB Webcam.

To add this to our robot in Viam, we need to:

1. Click on the `Config` tab on the robot we want to add the camera to. This will open up the configuration screen for the robot
1. Click the `+ Create Component` button to add a new component to the robot
1. Select `Camera` from the list of components
1. Select `Webcam` from the list of cameras
1. Give the camera a name, be sure to avoid spaces and special characters.
1. Choose `video0` for the Video Path
1. Click `Create Component`

![Rover robot](/assets/img/blog/elf/config.png){:class="img-fluid w-100 shadow-lg"}

---

### Add Data-Management Service

Next we need to add a data-management service to our robot. This will allow us to store the images we capture from the camera and use them to train our model.

To add a data-management service to our robot in Viam, we need to:

1. Click on the `Config` tab on the robot we want to add the camera to. This will open up the configuration screen for the robot
1. Click on the `Services` tab
1. Click on the `+ Create Service` button
1. Select `Data Management` from the list of services
1. Give the service a name (remember to avoid spaces and special characters)
1. Click the `Create` button
1. in the new `Data Management` service, review the directory the data (our images) will be captured to, its ok to leave this as the default `~/.viam/capture` folder
1. Notice the `Cloud sync` settings, we want this to be left on so that all the images we capture are sent to the Viam cloud service, as we will use this to quickly build our model.
1. Click the `Save config` button

![Rover robot](/assets/img/blog/elf/data.png){:class="img-fluid w-100 shadow-lg"}

---

### Add Data Capture Configuration

Next we need to add a data capture configuration to our robot. This will allow us to capture images from the camera and use them to train our model. We can set this to on or off, as we don't want to continually capture images, just when we want to train our model. 

To add a data capture configuration to our robot in Viam, we need to:

- In the Camera component we just created, click on the `Add method` button

- set the Type to `ReadImage` and the frequency to `0.3` (this will capture an image every 0.3 seconds)
- set the `Mine Type` to `image/jpeg`
- click on the `Save config` button at the bottom of the screen

---

## Capturing images

Now we have our robot configured, we can capture images from the camera and use them to train our model. To do this, we need to:

1. Position the robot so that the object we want to train it to recognise is in the centre of the camera's field of view
1. Move the object to different positions in the camera's field of view
1. The robot will capture a new image every 0.3 seconds, so we can move the object around and capture images from different angles, pretty quickly
1. Be sure to capture images from different angles, and different distances from the camera, in different lighting conditions, etc
1. Once we have captured enough images, we can use these to train our model. 
1. Go back to the Viam app, Select the robot from the list, and click on the `Config` tab
1. Click on the `Components` tab,  and then click on the `Off` button in the Camera -> Data capture configuration section. This will stop the robot from capturing images

---

## Prepare the images for training

The Viam software has a really easy to use training system. You can train a model to recognise objects in just a few minutes. I used the webcam on the robot to capture images of our elf from different angles, and then used the Viam software to train the model.

To do this:

1. log into [app.viam.com](https://app.viam.com)
1. Click on the `data` tab
1. In the `All Data` tab you will see a list of all the images that have been captured by your robot
1. Click on the first image

1. Click on the `Bounding box` button in the top right corner of the screen
1. Click and drag a box around the object you want to train the model to recognise
1. Click on the `Current label` button in the top right corner of the screen, and enter the name of the label (in this case `elf`)
1. Add this to a dataset, by clicking on the `Datasets` textbox and giving the dataset a name (in this case `elf-dataset`)
1. Repeat for all the images captured

![Viam create robot](/assets/img/blog/elf/label.png){:class="img-fluid w-50 shadow-lg"}

---

### Train the model from the dataset

Once we have labelled all the images, we can train the model from the dataset. To do this:

1. Click on the `Datasets` tab, within the `Data` tab
1. Click on the `elf-dataset` we created earlier
1. Click on the `Train model` button to star the model training process
1. The model will be trained using the images we captured earlier, this may take 5-10 minutes depending on the size of the dataset
1. Click on the `Models` tab to see the status of the model training process

---

### Deploying the model

We can send the trained model to our robot, so that it can use it to recognise objects. To do this:

1. Click on the `Fleet` tab, and select the robot to deploy the model to in the list
1. Click on the `config` tab, then the `Services` tab
1. Click on the `+ Create Service` button
1. Click on the `ML model` item in the list of services
1. Click on the `TFLite CPU` item in the list of models
1. Give the model a name (in this case `elf-model`)
1. Click on the `Deploy model on robot` button
1. Choose the `elf-model` we just created from the list of models
1. Click on the `Save config` button

![Viam create robot](/assets/img/blog/elf/ml_model.png){:class="img-fluid w-100 shadow-lg"}

![Viam create robot](/assets/img/blog/elf/deploy.png){:class="img-fluid w-100 shadow-lg"}

---

## Add Vision Service

Next we need to add a vision service to our robot. This will allow us to use the model we just trained to recognise objects. To do this:

1. Click on the `Config` tab on the robot we want to add the camera to. This will open up the configuration screen for the robot
1. Click on the `Services` tab
1. Click the `+ Create Service` button to add a new component to the robot
1. Select `Vision` from the list of services, then `ML model`
1. Give the service a name (remember to avoid spaces and special characters).
1. Click the `Create` button
1. in the new `Vision` service, choose the `elf-model` we just created from the list of models
1. Click the `Save config` button

---

### Evaluating the model

We can test the model to see how well it performs. To do this:

1. Click on the `Fleet` tab, and select the robot to deploy the model to in the list
1. Click on the `config` tab, then the `Components` tab
1. Click on the `+ Create Component` button
1. Click on the `Camera` item in the list of components, then `transform`
1. Give the component a name (in this case `elf-transform`)
1. Click on the `Save config button`
1. Add the following JSON to the `attributes` textbox:

   ```json
    {
        "pipeline": [
        {
            "attributes": {
            "detector_name": "elf-detector",
            "confidence_threshold": 0.5
            },
            "type": "detections"
        }
        ],
        "source": "cam"
    }
   ```

1. Click the `Save config` button

---

### Fine tuning the model

You can adjust the threshold confidence level to improve the accuracy of the model. To do this:

1. Change the value of the `confidence_threshold` attribute in the `elf-transform` component to `0.7` and see if the model detects the elf more accurately.

---

## Adding extra objects to the model

You can add extra objects to the model by capturing images of the new object, labelling them, and then training the model again.

Think about all the potential uses for this model. You could train it to recognise all sorts of objects, and then use it to control a robot to find them.
