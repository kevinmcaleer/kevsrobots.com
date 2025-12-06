---
title: Preparing Custom Object Detection Datasets
description: >-
    Learn how to collect, annotate, and organize images to create a high-quality custom object detection dataset
type: page
layout: lesson
---

## Overview

In this lesson, you'll learn how to prepare a custom object detection dataset from scratch. You'll discover best practices for collecting images, how to annotate them with bounding boxes, and how to organize your dataset for training. By the end of this lesson, you'll have a production-ready dataset for your custom objects.

---

## Objectives

By the end of this lesson, you will be able to:

- Collect diverse, high-quality images for your dataset
- Annotate objects with bounding boxes using LabelImg
- Organize datasets into proper train/test splits
- Understand dataset quality requirements
- Avoid common dataset preparation mistakes

---

## Why Dataset Quality Matters

Your model is only as good as your dataset. A well-prepared dataset with diverse examples will create a robust model that works in real-world conditions. A poor dataset leads to models that fail when conditions change.

**Real-world impact:**
- **Security camera**: Trained only on daytime images? Won't work at night
- **Warehouse robot**: Only annotated boxes from one angle? Fails with rotated boxes
- **Pet detector**: Only images of your white cat? Won't recognize your friend's black dog

Quality over quantity wins every time!

---

## Example Datasets by Use Case

Here are real-world examples to guide your dataset size:

| Use Case | Objects to Detect | Images Needed | Why |
|----------|------------------|----------------|-----|
| Smart doorbell | person, package, pet | 200-500 each | Varied people, lighting conditions |
| Warehouse robot | box, pallet, forklift | 300-600 each | Different angles, distances, stacking |
| Plant monitor | healthy_leaf, diseased_leaf, pest | 400-800 each | Many disease variations |
| Tool organizer | hammer, screwdriver, wrench | 150-300 each | Different brands, conditions |
{:class="table table-single"}

**Rule of thumb:** Start with 200-300 images per class. Add more if validation accuracy is low (<80%).

---

## Collecting Images

### Best Practices

**Diversity is key:**
1. **Multiple angles** - Top, side, 45 degrees, close-up, far away
2. **Lighting conditions** - Bright, dim, shadows, direct sunlight
3. **Backgrounds** - Clean, cluttered, indoor, outdoor
4. **Variations** - Different colors, sizes, orientations of objects
5. **Occlusions** - Partially hidden objects (real-world scenarios)

### Capture Strategy

For a coffee mug detector:

```bash
# Good dataset composition:
- 100 images: mug on desk from various angles
- 75 images: mug with hand holding it
- 50 images: mug partially hidden behind laptop
- 50 images: different lighting (morning, evening, artificial)
- 25 images: multiple mugs in frame
```

### Using Your Phone

Your smartphone is perfect for dataset collection:

1. **Take 50-100 photos** of each object
2. **Move around** the object - don't just rotate it
3. **Change lighting** - move near windows, turn lights on/off
4. **Add context** - place objects in realistic scenes
5. **Avoid blur** - hold steady or use timer

---

## Installing Annotation Tools

We'll use LabelImg - a simple, powerful tool for drawing bounding boxes:

```bash
# Install LabelImg
pip install labelImg
```

**If installation fails**, try the from-source method:

```bash
# Clone the repository
git clone https://github.com/HumanSignal/labelImg
cd labelImg

# Install dependencies
pip install PyQt5

# Run from source
python labelImg.py
```

---

## Hands-On: Annotate Your First Image

Let's walk through annotating a coffee mug:

### Step 1: Launch LabelImg

```bash
labelImg
```

### Step 2: Load Your Image Folder

1. Click **"Open Dir"** button
2. Navigate to your images folder
3. Click **"Change Save Dir"** and select the same folder (keeps annotations with images)

### Step 3: Set Annotation Format

1. Click **"View"** menu
2. Select **"Auto Save Mode"** (saves time!)
3. Ensure **PascalVOC** format is selected (not YOLO)

### Step 4: Draw Your First Bounding Box

1. Press **'W'** or click **"Create RectBox"**
2. Click and drag around the object
3. Type the label (e.g., "mug")
4. Press **Enter** to save

### Step 5: Check the Annotation File

LabelImg creates an XML file next to your image:

```xml
<annotation>
  <folder>images</folder>
  <filename>mug_001.jpg</filename>
  <size>
    <width>640</width>
    <height>480</height>
  </size>
  <object>
    <name>mug</name>
    <bndbox>
      <xmin>120</xmin>
      <ymin>80</ymin>
      <xmax>340</xmax>
      <ymax>420</ymax>
    </bndbox>
  </object>
</annotation>
```

**Your first annotation is complete!**

---

## Annotation Best Practices

### Tight Bounding Boxes

**Good:** Box tightly fits the object with minimal padding
```
┌──────┐
│ MUG │
└──────┘
```

**Bad:** Too much empty space
```
┌──────────┐
│          │
│   MUG   │
│          │
└──────────┘
```

### Handle Occlusions

If an object is partially hidden:
- **Draw box around visible parts only**
- **Label it anyway** (helps model learn partial detections)
- **Don't guess where hidden parts are**

### Multiple Objects

When annotating multiple objects:
- Create separate boxes for each
- Use the same label for same object types
- Press 'D' to go to next image
- Press 'A' to go to previous image

---

## Creating Dataset Structure

Organize your annotated dataset like this:

```
custom_dataset/
├── train/
│   ├── images/
│   │   ├── img_001.jpg
│   │   ├── img_002.jpg
│   │   └── ...
│   └── annotations/
│       ├── img_001.xml
│       ├── img_002.xml
│       └── ...
├── test/
│   ├── images/
│   │   ├── img_201.jpg
│   │   └── ...
│   └── annotations/
│       ├── img_201.xml
│       └── ...
└── labels.txt
```

**Create labels.txt:**
```
mug
laptop
keyboard
mouse
```

### Train/Test Split

**Common splits:**
- 80/20 split: 80% training, 20% testing
- 70/30 split: For smaller datasets (<500 images)

**Python script to split dataset:**

```python
import os
import shutil
import random

def split_dataset(source_dir, train_ratio=0.8):
    """Split images and annotations into train/test sets"""

    # Get all image files
    images = [f for f in os.listdir(source_dir) if f.endswith('.jpg')]
    random.shuffle(images)

    # Calculate split point
    split_idx = int(len(images) * train_ratio)
    train_images = images[:split_idx]
    test_images = images[split_idx:]

    # Create directories
    for split in ['train', 'test']:
        os.makedirs(f'{split}/images', exist_ok=True)
        os.makedirs(f'{split}/annotations', exist_ok=True)

    # Copy files
    for img in train_images:
        shutil.copy(f'{source_dir}/{img}', f'train/images/{img}')
        xml = img.replace('.jpg', '.xml')
        shutil.copy(f'{source_dir}/{xml}', f'train/annotations/{xml}')

    for img in test_images:
        shutil.copy(f'{source_dir}/{img}', f'test/images/{img}')
        xml = img.replace('.jpg', '.xml')
        shutil.copy(f'{source_dir}/{xml}', f'test/annotations/{xml}')

    print(f"✓ Split complete!")
    print(f"  Training images: {len(train_images)}")
    print(f"  Testing images: {len(test_images)}")

# Use it:
split_dataset('my_images')
```

---

## Try It Yourself

1. **Collect 50 images** of an object you want to detect (use your phone!)
2. **Annotate all 50 images** using LabelImg - aim for <30 seconds per image
3. **Create train/test split** - 40 training, 10 testing
4. **Check annotation quality** - Open a few XML files and verify coordinates

**Challenge:** Annotate a complex scene with 3+ different objects. Give each object type a different label.

---

## Common Issues

**Problem**: "ModuleNotFoundError: No module named 'PyQt5'"
**Solution**: Install PyQt5: `pip install PyQt5`
**Why**: LabelImg requires PyQt5 for its graphical interface

**Problem**: Annotations saved in wrong format (YOLO instead of PascalVOC)
**Solution**: In LabelImg menu, click "View" → ensure "PascalVOC" is selected
**Why**: Different frameworks use different annotation formats. We need PascalVOC (XML) for PyTorch

**Problem**: Can't see bounding boxes clearly
**Solution**: Use 'Ctrl+Plus' to zoom in, click and drag box corners to adjust
**Why**: Precision matters - boxes should tightly fit objects for best model accuracy

**Problem**: LabelImg crashes when opening images
**Solution**: Ensure images are valid JPG/PNG files, try converting with an image tool
**Why**: Corrupted or unusual image formats can cause crashes

**Problem**: Lost all my annotations!
**Solution**: Always enable "Auto Save" mode and check XML files are being created alongside images
**Why**: Manual saving can be forgotten, especially when annotating hundreds of images

---

## Dataset Quality Checklist

Before moving to training, verify:

- [ ] At least 200 images per object class
- [ ] Images show objects in varied conditions (angles, lighting, backgrounds)
- [ ] All bounding boxes are tight around objects
- [ ] Annotation files exist for every image
- [ ] No corrupted or duplicate images
- [ ] Train/test split is complete
- [ ] Labels are consistent (no typos: "mug" vs "cup")

---

## Summary

You've learned:
- Why dataset quality determines model performance
- How to collect diverse, real-world images
- Installing and using LabelImg for annotation
- Creating tight, accurate bounding boxes
- Organizing datasets with proper train/test splits
- Common pitfalls and how to avoid them

You now have a professional-quality dataset ready for training!

---

## Next Lesson

In the next lesson, you'll use your annotated dataset to train a custom PyTorch object detection model using transfer learning.

---
