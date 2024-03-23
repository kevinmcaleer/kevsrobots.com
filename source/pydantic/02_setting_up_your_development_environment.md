---
title: Setting Up Your Environment
description: Learn how to set up your development environment for Pydantic, including installing Python, Pydantic, and creating your first simple data model.
layout: lesson
type: page
cover: assets/2.png
---

![Pydantic Setup]({{ page.cover }}){:class="cover"}

## Prerequisites

Before diving into Pydantic, you should have a basic understanding of Python and programming concepts. Additionally, ensure you have the latest version of Python installed on your machine. Pydantic requires Python 3.6 or higher.

---

## Installing Pydantic

Pydantic can be installed using pip, Python's package installer. Open your terminal or command prompt and run the following command:

```bash
pip install pydantic
```

This command installs Pydantic and its dependencies, preparing your environment for developing with Pydantic.

---

## Setting Up a Virtual Environment

It's a best practice to use a virtual environment for your Python projects. This keeps your project's dependencies separate from other projects and the system-wide Python installation. You can create a virtual environment using the following commands:

```bash
python -m venv myprojectenv
source myprojectenv/bin/activate  # On Unix/macOS
myprojectenv\Scripts\activate.bat # On Windows
```

Once activated, your command line will indicate that you're inside the virtual environment. Now, you can install Pydantic within this environment.

---

## Creating Your First Pydantic Model

A Pydantic model defines the structure of your data, including the types and validation rules. Here's a simple example:

```python
from pydantic import BaseModel

class User(BaseModel):
    name: str
    age: int
    is_active: bool = True

# Creating an instance of the User model
user = User(name="John Doe", age=30)

print(user)
```

This code defines a `User` model with three fields: `name`, `age`, and `is_active`. The `is_active` field is optional and defaults to `True` if not provided.

---

## Testing Your Setup

To test your setup, save the model code in a file named `test_pydantic.py` and run it using Python:

```bash
python test_pydantic.py
```

You should see the output displaying the `User` object, indicating that your environment is correctly set up and ready for developing with Pydantic.

---

## Additional Resources

- [Pydantic Documentation](https://pydantic-docs.helpmanual.io/)
- [Python Virtual Environments](https://docs.python.org/3/tutorial/venv.html)

---

## Lesson Assignment

Create a virtual environment, install Pydantic, and define a simple model representing a blog post with fields for the title, content, and publication date. Test your model by creating an instance of it and printing the instance.

---
