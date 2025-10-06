---
title: Post-processing ONNX Models
description: >-
    Learn how to post-process ONNX models using the `imxconv-pt` tool. You will learn how to convert compressed ONNX models to the OpenVINO Intermediate Representation (IR) format.
type: page
layout: lesson
---

We need to convert our compressed ONNX model to the OpenVINO Intermediate Representation (IR) format. The `imxconv-pt` tool is used to post-process ONNX models. The tool is available in the `imxconv-pt` package, which can be installed using the following command:

```bash
pip install 'imx500-converter[pt]'
```

The `imxconv-pt` tool can be used to convert compressed ONNX models to the OpenVINO Intermediate Representation (IR) format. The tool takes the following arguments:

```bash
imxconv-pt -i <compressed ONNX model> -o <output folder>
```

The `-i` argument specifies the path to the compressed ONNX model, and the `-o` argument specifies the output folder where the converted IR model will be saved.

---

## On the Raspberry Pi with the AI Camera attached

To install the `imx500-tools` package, run the following command:

```bash
sudo apt install imx500-tools
```

The `imx500-tools` package contains the `imxconv-pt` tool, which can be used to post-process ONNX models.



---

## Converting the package for use on the Raspberry Pi AI Camera

To convert the package for use on the Raspberry Pi AI Camera, run the following command:

```bash
imx500-package.sh -i <path to packerOut.zip> -o <output folder>
```