---
title: Training Custom Object Detection Models
description: >-
    Train a custom PyTorch object detection model using transfer learning with Faster R-CNN to detect your own objects
type: page
layout: lesson
---

## Overview

Now comes the exciting part - training your model! In this lesson, you'll use PyTorch and transfer learning to create a custom object detector. You'll start with a pre-trained Faster R-CNN model and fine-tune it on your dataset. By the end, you'll have a working model that can detect your custom objects.

---

## Objectives

By the end of this lesson, you will be able to:

- Understand transfer learning and why it's powerful
- Load and prepare your dataset for PyTorch
- Modify a pre-trained Faster R-CNN model for custom classes
- Train the model with proper monitoring
- Evaluate model performance
- Save your trained model

---

## Why Transfer Learning?

Training a detection model from scratch requires millions of images and weeks of compute time. Transfer learning lets you leverage a model pre-trained on millions of images and adapt it to your specific objects in hours.

**Real-world analogy:**
Instead of learning to recognize objects from birth, you're teaching an expert photographer to recognize your specific items. They already know what edges, shapes, and textures are - they just need to learn your unique classes.

**Benefits:**
- **Faster training** - Hours instead of weeks
- **Less data needed** - 200-500 images instead of millions
- **Better accuracy** - Pre-trained features generalize well
- **Lower compute cost** - Works on laptop CPUs (though GPU is faster)

---

## Install Required Packages

First, install PyTorch and related libraries:

```bash
# Install PyTorch (CPU version - works on any computer)
pip install torch torchvision

# Or for NVIDIA GPU support (much faster training)
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118

# Install additional tools
pip install matplotlib pillow tqdm
```

**Verify installation:**

```python
import torch
import torchvision

print(f"✓ PyTorch version: {torch.__version__}")
print(f"✓ Torchvision version: {torchvision.__version__}")
print(f"✓ CUDA available: {torch.cuda.is_available()}")

if torch.cuda.is_available():
    print(f"✓ GPU: {torch.cuda.get_device_name(0)}")
else:
    print("  Using CPU (training will be slower)")
```

---

## Create Dataset Loader

PyTorch needs a custom dataset class to load your images and annotations:

```python
import torch
from torch.utils.data import Dataset
from PIL import Image
import xml.etree.ElementTree as ET
import os

class CustomDataset(Dataset):
    """Dataset loader for object detection with PascalVOC annotations"""

    def __init__(self, image_dir, annotation_dir, transforms=None):
        self.image_dir = image_dir
        self.annotation_dir = annotation_dir
        self.transforms = transforms
        self.images = [f for f in os.listdir(image_dir) if f.endswith('.jpg')]

    def __len__(self):
        return len(self.images)

    def __getitem__(self, idx):
        # Load image
        img_name = self.images[idx]
        img_path = os.path.join(self.image_dir, img_name)
        image = Image.open(img_path).convert("RGB")

        # Load annotation
        xml_name = img_name.replace('.jpg', '.xml')
        xml_path = os.path.join(self.annotation_dir, xml_name)

        boxes = []
        labels = []

        tree = ET.parse(xml_path)
        root = tree.getroot()

        for obj in root.findall('object'):
            label = obj.find('name').text
            bbox = obj.find('bndbox')

            xmin = int(bbox.find('xmin').text)
            ymin = int(bbox.find('ymin').text)
            xmax = int(bbox.find('xmax').text)
            ymax = int(bbox.find('ymax').text)

            boxes.append([xmin, ymin, xmax, ymax])
            labels.append(self.class_to_idx[label])

        # Convert to tensors
        boxes = torch.as_tensor(boxes, dtype=torch.float32)
        labels = torch.as_tensor(labels, dtype=torch.int64)

        target = {
            'boxes': boxes,
            'labels': labels,
            'image_id': torch.tensor([idx])
        }

        if self.transforms:
            image = self.transforms(image)

        return image, target

    # Map class names to indices
    class_to_idx = {
        'background': 0,
        'mug': 1,
        'laptop': 2,
        'keyboard': 3,
        # Add your classes here
    }
```

---

## Complete Training Script

Here's a complete, production-ready training script:

```python
import torch
import torchvision
from torchvision.models.detection import fasterrcnn_resnet50_fpn
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torch.utils.data import DataLoader
import time
from tqdm import tqdm

# ===== Configuration =====
NUM_CLASSES = 4  # Background + 3 object classes
NUM_EPOCHS = 50
BATCH_SIZE = 4
LEARNING_RATE = 0.005

# ===== Load Dataset =====
print("Loading dataset...")
train_dataset = CustomDataset(
    image_dir='custom_dataset/train/images',
    annotation_dir='custom_dataset/train/annotations'
)

train_loader = DataLoader(
    train_dataset,
    batch_size=BATCH_SIZE,
    shuffle=True,
    num_workers=4,
    collate_fn=lambda x: tuple(zip(*x))
)

print(f"✓ Loaded {len(train_dataset)} training images")

# ===== Create Model =====
print("Creating model...")

# Load pre-trained Faster R-CNN
model = fasterrcnn_resnet50_fpn(pretrained=True)

# Replace the classifier head for our custom classes
in_features = model.roi_heads.box_predictor.cls_score.in_features
model.roi_heads.box_predictor = FastRCNNPredictor(in_features, NUM_CLASSES)

# Move model to GPU if available
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model.to(device)

print(f"✓ Model loaded on {device}")

# ===== Setup Optimizer =====
params = [p for p in model.parameters() if p.requires_grad]
optimizer = torch.optim.SGD(
    params,
    lr=LEARNING_RATE,
    momentum=0.9,
    weight_decay=0.0005
)

# Learning rate scheduler (reduces LR over time)
lr_scheduler = torch.optim.lr_scheduler.StepLR(
    optimizer,
    step_size=15,
    gamma=0.1
)

# ===== Training Loop =====
print(f"\nStarting training for {NUM_EPOCHS} epochs...")
print("=" * 50)

model.train()
losses_per_epoch = []
start_time = time.time()

for epoch in range(NUM_EPOCHS):
    epoch_loss = 0
    progress_bar = tqdm(train_loader, desc=f"Epoch {epoch+1}/{NUM_EPOCHS}")

    for images, targets in progress_bar:
        # Move data to device
        images = [img.to(device) for img in images]
        targets = [{k: v.to(device) for k, v in t.items()} for t in targets]

        # Forward pass
        loss_dict = model(images, targets)
        losses = sum(loss for loss in loss_dict.values())

        # Backward pass
        optimizer.zero_grad()
        losses.backward()
        optimizer.step()

        # Track loss
        epoch_loss += losses.item()
        progress_bar.set_postfix({'loss': f'{losses.item():.4f}'})

    # Calculate average loss for epoch
    avg_loss = epoch_loss / len(train_loader)
    losses_per_epoch.append(avg_loss)

    # Update learning rate
    lr_scheduler.step()

    # Print epoch summary
    elapsed = time.time() - start_time
    print(f"\nEpoch {epoch+1}/{NUM_EPOCHS}")
    print(f"  Average Loss: {avg_loss:.4f}")
    print(f"  Time Elapsed: {elapsed/60:.1f} min")
    print(f"  Learning Rate: {optimizer.param_groups[0]['lr']:.6f}")
    print("-" * 50)

# ===== Save Model =====
torch.save(model.state_dict(), 'custom_detector.pth')
print(f"\n✓ Training complete!")
print(f"✓ Model saved to 'custom_detector.pth'")
print(f"✓ Total time: {elapsed/60:.1f} minutes")
```

**Expected training times:**
- MacBook Pro M1 (GPU): ~1 hour for 50 epochs
- Mac/PC (CPU only): ~3-4 hours for 50 epochs
- Linux with NVIDIA RTX 3080: ~30 minutes for 50 epochs

---

## Monitor Training Progress

Add visualization to track how well training is going:

```python
import matplotlib.pyplot as plt

# After training completes, plot the loss curve
plt.figure(figsize=(10, 5))
plt.plot(losses_per_epoch, linewidth=2)
plt.xlabel('Epoch', fontsize=12)
plt.ylabel('Loss', fontsize=12)
plt.title('Training Loss Over Time', fontsize=14)
plt.grid(True, alpha=0.3)
plt.savefig('training_loss.png', dpi=150, bbox_inches='tight')
plt.show()

print("✓ Loss plot saved to 'training_loss.png'")
```

**What to look for:**
- **Good:** Smooth downward curve that plateaus
- **Bad:** Loss stays flat → learning rate too low
- **Bad:** Loss jumps around wildly → learning rate too high
- **Bad:** Loss increases → data problem or bug

---

## Test Your Trained Model

Let's verify the model works:

```python
from PIL import Image
import torchvision.transforms as T

# Load trained model
model = fasterrcnn_resnet50_fpn(pretrained=False, num_classes=NUM_CLASSES)
model.load_state_dict(torch.load('custom_detector.pth'))
model.eval()
model.to(device)

# Load test image
test_image = Image.open('custom_dataset/test/images/test_001.jpg')
transform = T.ToTensor()
image_tensor = transform(test_image).to(device)

# Run inference
with torch.no_grad():
    predictions = model([image_tensor])

# Print results
boxes = predictions[0]['boxes'].cpu().numpy()
labels = predictions[0]['labels'].cpu().numpy()
scores = predictions[0]['scores'].cpu().numpy()

print(f"\nDetections on test image:")
for box, label, score in zip(boxes, labels, scores):
    if score > 0.5:  # Only show confident detections
        print(f"  Class {label}: {score:.2f} - Box: {box}")
```

---

## Try It Yourself

1. **Modify hyperparameters** - Try `LEARNING_RATE = 0.001` or `NUM_EPOCHS = 100`
2. **Add data augmentation** - Flip images horizontally during training
3. **Track validation loss** - Create a validation dataset and monitor overfitting

**Challenge:** Implement early stopping that saves the best model when validation loss stops improving.

---

## Common Issues

**Problem**: "CUDA out of memory" error
**Solution**: Reduce `BATCH_SIZE` to 2 or 1, or use CPU
**Why**: GPU has limited RAM, smaller batches use less memory

**Problem**: Loss is NaN or very high (>100)
**Solution**: Check annotations are correct, reduce learning rate to 0.001
**Why**: Bad data or learning rate too high causes numerical instability

**Problem**: Training is very slow (>8 hours)
**Solution**: Use GPU, reduce image resolution, or use fewer epochs
**Why**: CPU training is 10-20x slower than GPU

**Problem**: Model detects nothing on test images
**Solution**: Train for more epochs, check if classes match between train/test
**Why**: Model hasn't learned enough, or label mismatch

**Problem**: "KeyError" when loading annotations
**Solution**: Verify all images have matching XML files with correct class names
**Why**: Dataset loader can't find expected files or class names

---

## Summary

You've learned:
- How transfer learning accelerates training
- Creating PyTorch dataset loaders for object detection
- Modifying pre-trained models for custom classes
- Training with proper monitoring and learning rate scheduling
- Visualizing training progress
- Testing your trained model

You now have a trained object detection model! In the next lesson, you'll convert it to ONNX format for deployment on the Raspberry Pi.

---

## Next Lesson

In the next lesson, you'll learn how to convert your PyTorch model to ONNX format for efficient deployment on edge devices like the Raspberry Pi AI Camera.

---
