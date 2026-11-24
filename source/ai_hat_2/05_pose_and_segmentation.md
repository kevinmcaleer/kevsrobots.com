---
title: Pose and segmentation
description: Run YOLOv8 pose estimation and YOLOv5 segmentation on the camera
layout: lesson
type: page
cover: /learn/ai_hat_2/assets/banners/05_pose_and_segmentation.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Detecting a person is useful. Knowing how they are standing is better. The docs list two more camera demos that work on the HAT.

## Pose estimation

The docs describe this as YOLOv8 with a 17-point human pose:

```bash
rpicam-hello -t 0 --post-process-file /usr/share/rpi-camera-assets/hailo_yolov8_pose.json
```

You should see a skeleton drawn over each person in view, with points for the body joints. Raise an arm and watch it follow.

**What it does:** finds each person and the position of 17 body points on them.

**Real robot example:** a gesture-controlled robot. If the point for the wrist is above the point for the shoulder, the person has a hand raised, so the robot can stop, wave back or follow. The detection from lesson 4 only tells you "a person is here". Pose tells you what they are doing.

```python
# Pretend pose data: y grows DOWN the image, so a smaller y is higher up.
wrist_y = 0.20
shoulder_y = 0.45

if wrist_y < shoulder_y:
    print("Hand is raised: stop and wait")
else:
    print("Hand is down: carry on")
```

This is invented example data, to show the idea. In lesson 6 we look at how to get real data out.

## Segmentation

Instead of a box, segmentation colours in the exact shape of each object. The docs give this command for YOLOv5 segmentation with a colour mask:

```bash
rpicam-hello -t 0 --post-process-file /usr/share/rpi-camera-assets/hailo_yolov5_segmentation.json --framerate 20
```

**Real robot example:** a box tells you roughly where a person is. A mask tells you the exact outline, which could help a robot arm decide where to reach, or a mobile robot decide whether a gap is wide enough.

## Which to use

| Task | Use | Cost |
|---|---|---|
| Is something there? | Object detection | Cheapest |
| What is that person doing? | Pose | Needs people in view |
| Exactly which pixels? | Segmentation | The docs example sets `--framerate 20` |
{:class="table table-single"}

## Try it Yourself

1. Run the pose demo and try three poses: arms up, arms out, one arm up.
2. Run the segmentation demo and look at how the mask follows your outline.
3. Challenge: decide which pose you'd use as a "stop" signal for a robot, and why it should be easy to tell apart from walking.

## Common Issues

**Problem**: The skeleton jumps about.
**Solution**: Improve the lighting and keep the whole body in frame.
**Why**: The model has to guess joints it cannot see, and a poorly lit frame gives it less to work with.

**Problem**: Segmentation looks choppy.
**Solution**: Keep the `--framerate 20` option from the docs example, and check the HAT has its heatsink and fan.
**Why**: The docs example asks for 20 frames per second, and a hot chip may slow down.
