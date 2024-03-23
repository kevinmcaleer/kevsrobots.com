---
layout: lesson
title: Advanced Validation Techniques
author: Kevin McAleer
type: page
cover: assets/4.png
date: 2024-03-23
previous: 03_basic_data_models.html
next: 05_custom_data_types.html
description: Dive deeper into Pydantic's capabilities by learning how to implement
  custom validation logic for your data models.
percent: 32
duration: 2
navigation:
- name: Mastering Pydantic for Robust Data Validation
- content:
  - section: Introduction to Pydantic
    content:
    - name: Overview of Pydantic and Data Validation
      link: 01_overview_of_pydantic.html
    - name: Setting Up Your Environment
      link: 02_setting_up_your_development_environment.html
    - name: Basic Data Models
      link: 03_basic_data_models.html
  - section: Advanced Data Validation with Pydantic
    content:
    - name: Advanced Validation Techniques
      link: 04_advanced_validation_techniques.html
    - name: Working with Custom Data Types
      link: 05_custom_data_types.html
    - name: Managing Recursive Models
      link: 06_recursive_models.html
  - section: Pydantic with FastAPI
    content:
    - name: Integrating Pydantic with FastAPI
      link: 07_integrating_pydantic_with_fastapi.html
    - name: Advanced Features in FastAPI
      link: 08_request_validation_and_serialization.html
    - name: Building a CRUD Application with FastAPI
      link: 09_building_a_crud_application.html
  - section: Best Practices and Real-world Applications
    content:
    - name: Deploying Your FastAPI & Pydantic App
      link: 10_pydantic_best_practices.html
    - name: Pydantic Best Practices for FastAPI Applications
      link: 11_performance_optimization_tips.html
    - name: Performance Optimization in FastAPI and Pydantic Applications
      link: 12_case_studies_and_real_world_applications.html
---


![Pydantic Advanced Validation]({{ page.cover }}){:class="cover"}

## Custom Validators in Pydantic

While type annotations and basic validators cover many common use cases, Pydantic also allows for custom validation logic through the use of decorators. This enables you to define your own rules for data validation, transformation, and normalization.

---

## Using the `validator` Decorator

Pydantic's `validator` decorator allows you to attach custom validation functions to model fields. These functions can perform additional checks and transformations on field values.

```python
from pydantic import BaseModel, validator

class Product(BaseModel):
    name: str
    description: str
    price: float
    discount_price: float

    @validator('discount_price')
    def check_discount_price(cls, v, values, **kwargs):
        if 'price' in values and v >= values['price']:
            raise ValueError('discount_price must be less than the price')
        return v
```

In this example, a `Product` model is defined with a custom validator for the `discount_price` field. The validator ensures that the `discount_price` is always less than the `price`.

---

## Pre and Post Validators

Validators can be set to run either before (`pre=True`) or after Pydantic's standard validation. Pre-validators are useful for data transformation or normalization, while post-validators can enforce additional constraints on validated data.

```python
@validator('signup_ts', pre=True)
def parse_signup_ts(cls, v):
    # Example pre-validator to parse a string into a datetime
    return parse_date_string(v)
```

---

## Complex Field Validation

Validators are not limited to simple conditions. They can incorporate complex logic, accessing other fields in the model via the `values` argument, and can even raise multiple errors.

---

## Sharing Validators

Validators can be shared between models or fields by defining them as standalone functions and using the `each_item=True` parameter for iterable fields.

---

## Lesson Assignment

Create a `User` model with fields for `username`, `email`, and `age`. Implement custom validators that:

- Ensure the `username` is at least 3 characters long.
- Validate that the `email` follows a simple pattern (e.g., contains `@`).
- Check that the `age` is between 18 and 100.

Experiment with both pre and post validators to familiarize yourself with their differences and applications.

---

## Additional Resources

- [Pydantic Validators Documentation](https://pydantic-docs.helpmanual.io/usage/validators/)
- [Python Regular Expressions for Email Validation](https://docs.python.org/3/library/re.html)

---
