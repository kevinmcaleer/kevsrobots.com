---
layout: lesson
title: Working with Custom Data Types
author: Kevin McAleer
type: page
cover: assets/5.png
date: 2024-03-23
previous: 04_advanced_validation_techniques.html
next: 06_recursive_models.html
description: Learn to extend Pydantic's functionality by defining and using custom
  data types for more granular control over your data validation.
percent: 40
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


![Pydantic Custom Data Types]({{ page.cover }}){:class="cover"}

## Extending Pydantic with Custom Data Types

Pydantic's powerful validation system can be extended with custom data types, enabling you to define bespoke validation logic and serialization/deserialization rules for specific data structures.

---

## Defining Custom Data Types

A custom data type in Pydantic is usually defined by extending existing base types and adding custom validation or transformation logic. Here's how you can create a custom type for handling email addresses:

```python
from pydantic import BaseModel, EmailStr

class Email(EmailStr):
    @classmethod
    def __get_validators__(cls):
        yield cls.validate

    @classmethod
    def validate(cls, v):
        # Place additional custom validation logic here
        if not "@" in v:
            raise ValueError("Invalid email address")
        return v.lower()
```

This example shows a custom `Email` type that extends `EmailStr`, a built-in Pydantic type for email validation. It adds additional logic to ensure all emails are converted to lowercase.

---

## Using Custom Data Types in Models

Once you've defined a custom data type, you can use it in your models just like any built-in type.

```python
class User(BaseModel):
    name: str
    email: Email  # Using the custom Email type
```

In this model, the `email` field uses the custom `Email` data type, applying both the built-in validation from `EmailStr` and the additional custom logic.

---

## Advantages of Custom Data Types

- **Flexibility:** Tailor validation logic to the specific needs of your application.
- **Reusability:** Define once and use across multiple models, ensuring consistency.
- **Clarity:** Improve code readability by using semantically meaningful types.

---

## Lesson Assignment

Define a custom data type for `PhoneNumber` that:

- Validates that the input is a string formatted as a phone number (e.g., starts with "+" followed by country code and number).
- Normalizes the phone number by removing spaces and dashes.

Use this custom type in a `Contact` model that includes fields for `name` and `phone_number`. Test your model with various phone number formats to ensure your validation and normalization logic works as expected.

---

## Additional Resources

- [Pydantic Custom Types Documentation](https://pydantic-docs.helpmanual.io/usage/types/#custom-data-types)
- [Python Regular Expressions for Pattern Matching](https://docs.python.org/3/library/re.html)

---
