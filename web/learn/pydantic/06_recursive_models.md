---
layout: lesson
title: Managing Recursive Models
author: Kevin McAleer
type: page
cover: assets/6.png
date: 2024-03-23
previous: 05_custom_data_types.html
next: 07_integrating_pydantic_with_fastapi.html
description: Explore how to effectively define and work with recursive models in Pydantic,
  enabling sophisticated data structures like trees and nested objects.
percent: 48
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


![Pydantic Recursive Models]({{ page.cover }}){:class="cover"}

## Understanding Recursive Models

Recursive models allow for the definition of complex, nested data structures within Pydantic. These are particularly useful for representing hierarchical data, such as organizational charts, file systems, or any nested relationships.

---

## Defining Recursive Models

To define a recursive model in Pydantic, a model may reference itself within its fields. Pydantic supports this natively, but it requires forward declaration for self-referencing models, using a string to indicate the model's name before it's fully defined.

```python
from typing import List, Optional
from pydantic import BaseModel

class TreeNode(BaseModel):
    name: str
    children: Optional[List['TreeNode']] = None
```

In this example, `TreeNode` is a model representing a node in a tree. Each node can have multiple children, which are also instances of `TreeNode`, demonstrating the recursive nature of the model.

---

## Working with Recursive Models

Recursive models are instantiated and used in the same way as any other Pydantic model. However, when working with deeply nested data, consider the complexity and potential performance implications.

```python
tree = TreeNode(
    name="root",
    children=[
        TreeNode(name="child1"),
        TreeNode(name="child2", children=[TreeNode(name="grandchild1")]),
    ],
)

print(tree)
```

This code creates a simple tree structure with a root node, two child nodes, and one grandchild node, showcasing the recursive model in action.

---

## Handling Circular References

Pydantic's `parse_obj` method can be used to handle circular references or deeply nested structures by providing a dictionary representation of the data. This method ensures that instances are correctly created without exceeding Python's recursion limit.

---

## Lesson Assignment

Define a recursive model `Category` for representing a category tree where each category can have multiple subcategories. Each `Category` should have fields for `id`, `name`, and `subcategories` (optional, a list of `Category`).

Create an instance of your `Category` model representing a simple category structure with at least one subcategory and test its instantiation and serialization capabilities.

---

## Additional Resources

- [Pydantic Documentation on Recursive Models](https://pydantic-docs.helpmanual.io/usage/models/#recursive-models)
- [Python's Type Hints for Recursive Type](https://docs.python.org/3/library/typing.html#typing.ForwardRef)

---
