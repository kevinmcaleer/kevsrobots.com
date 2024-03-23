---
title: Basic Data Models
description: A guide to creating your first data models with Pydantic, understanding field types, validation, and setting default values.
layout: lesson
type: page
cover: assets/3.png
---

![Pydantic Data Models]({{ page.cover }}){:class="cover"}

## Introduction to Pydantic Models

Pydantic models are the core of Pydantic, allowing for easy data validation and settings management through Python type annotations. These models define the structure of your data, enforce type constraints, and can automatically convert and validate data.

---

## Creating a Basic Model

To define a Pydantic model, you create a class that inherits from `BaseModel`. Each attribute of the class represents a field in the model, with its type annotation defining the expected type.

```python
from pydantic import BaseModel

class Item(BaseModel):
    name: str
    description: str = None
    price: float
    tax: float = None
```

In this example, `Item` is a simple model representing an item in an inventory. It includes a name, description, price, and tax. The `description` and `tax` fields are optional and default to `None`.

---

## Field Types and Validation

Pydantic supports a wide range of field types, including `int`, `float`, `str`, `bool`, and more. It also supports more complex types like `datetime` and `List[T]` for lists of items of type `T`.

When you create an instance of a Pydantic model, Pydantic validates the input data against the model's field types. If the data doesn't match the expected types, Pydantic raises a validation error.

```python
item = Item(name="Laptop", price=1500.00, tax=0.15)
```

This code creates an instance of the `Item` model. Pydantic validates that `name` is a string, `price` is a float, and `tax` is a float (and optionally `None`).

---

## Default Values and Optional Fields

You can define default values for fields by assigning values to the class attributes. If an instance is created without those fields, Pydantic uses the default values.

Fields without default values are considered required. To make a field optional, you can use `typing.Optional[T]` or simply assign a default value of `None`.

```python
from typing import Optional

class Item(BaseModel):
    name: str
    description: Optional[str] = None
    price: float
    tax: Optional[float] = None
```

---

## Lesson Assignment

Create a Pydantic model for a `User` that includes the following fields: `username`, `email`, `signup_ts` (signup timestamp, optional), and `friends` (list of usernames, optional). Experiment by creating instances of your model with different data to see how Pydantic handles validation and defaults.

---

## Additional Resources

- [Pydantic Field Types](https://pydantic-docs.helpmanual.io/usage/types/)
- [Pydantic Validators](https://pydantic-docs.helpmanual.io/usage/validators/)

---
