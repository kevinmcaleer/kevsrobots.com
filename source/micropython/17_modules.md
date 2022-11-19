---
title: Modules
description: Learn about the built-in Modules
layout: lesson
---

## Modules

MicroPython has many modules that are built into the firmware. We still need to `import` these if we want to use them in our code.

This is because when we `import` a modules MicroPython has to read all the lines of code in that modules and store the functions in `RAM` (Random Access Memory). MicroPython devices have limited `RAM` so we need to ensure we only import the libraries we need for each specific program.

Lets have alook at all the modules on our MicroPython device. 

**Example**

1. Type the following directly into the `REPL`:

```python
help('modules')
```

![Screenshot of all the installed Modules](assets/modules.png){:class="img-fluid w-100 shadow-lg"}