---
title: Advanced Validation Techniques
description: Dive deeper into Pydantic's capabilities by learning how to implement custom validation logic for your data models.
layout: lesson
type: page
cover: assets/4.png
---

![Pydantic Advanced Validation]({{ page.cover }}){:class="cover"}

## Custom Validators in Pydantic

While type annotations and basic validators cover many common use cases, Pydantic also allows for custom validation logic through the use of decorators. This enables you to define your own rules for data validation, transformation, and normalization.

---

## Using the `validator` Decorator

Pydantic's `validator` decorator allows you to attach custom validation functions to model fields. These functions can perform additional checks and transformations on field values.

```python
from pydantic import BaseModel, field_validator, ValidationInfo

class Product(BaseModel):
    name: str
    description: str
    price: float
    discount_price: float

    @field_validator('discount_price')
    def check_discount_price(cls, v: str, info: ValidationInfo):
        if 'price' in info.data and v >= info.data['price']:
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
