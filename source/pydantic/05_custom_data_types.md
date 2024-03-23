---
title: Working with Custom Data Types
description: Learn to extend Pydantic's functionality by defining and using custom data types for more granular control over your data validation.
layout: lesson
type: page
cover: assets/5.png
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
