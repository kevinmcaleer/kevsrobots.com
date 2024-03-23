---
layout: lesson
title: Setting Up Your Environment
author: Kevin McAleer
type: page
cover: assets/2.png
date: 2024-03-23
previous: 01_overview_of_pydantic.html
next: 03_basic_data_models.html
description: Learn how to set up your development environment for Pydantic, including
  installing Python, Pydantic, and creating your first simple data model.
percent: 16
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
