---
layout: lesson
title: Building a CRUD Application with FastAPI
author: Kevin McAleer
type: page
cover: assets/9.png
date: 2024-03-23
previous: 08_request_validation_and_serialization.html
next: 10_pydantic_best_practices.html
description: Step-by-step guide to creating a basic CRUD application using FastAPI
  and Pydantic, covering database integration, endpoint creation, and testing.
percent: 72
duration: 3
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


![Building a CRUD Application with FastAPI and Pydantic]({{ page.cover }}){:class="cover"}

## Introduction to CRUD Applications

CRUD applications are foundational to web development, allowing users to create, read, update, and delete resources. This lesson will demonstrate how to build a simple CRUD API for a resource like `Item` using FastAPI and Pydantic, including database integration and API testing.

---

## Setting Up Your Project

Start by setting up a new FastAPI project and create a Pydantic model to represent the `Item` resource:

```python
from pydantic import BaseModel

class Item(BaseModel):
    name: str
    description: str = None
    price: float
    in_stock: bool = True
```

---

## Database Integration

For simplicity, we'll use an in-memory dictionary as our database. In a real-world scenario, you might integrate with SQL or NoSQL databases using ORMs or database drivers.

```python
# This will act as our fake database
items_db = {}
```

---

## Creating CRUD Endpoints

Implement the CRUD operations as FastAPI endpoints. Each operation corresponds to a standard HTTP method: POST for create, GET for read, PUT for update, and DELETE for delete.

---

### Create an Item

```python
from fastapi import FastAPI, HTTPException

app = FastAPI()

@app.post("/items/")
async def create_item(item: Item):
    if item.name in items_db:
        raise HTTPException(status_code=400, detail="Item already exists")
    items_db[item.name] = item
    return item
```

---

### Read an Item

```python
@app.get("/items/{item_name}")
async def read_item(item_name: str):
    if item_name not in items_db:
        raise HTTPException(status_code=404, detail="Item not found")
    return items_db[item_name]
```

---

### Update an Item

```python
@app.put("/items/{item_name}")
async def update_item(item_name: str, item: Item):
    if item_name not in items_db:
        raise HTTPException(status_code=404, detail="Item not found")
    items_db[item_name] = item
    return item
```

---

### Delete an Item

```python
@app.delete("/items/{item_name}")
async def delete_item(item_name: str):
    if item_name not in items_db:
        raise HTTPException(status_code=404, detail="Item not found")
    del items_db[item_name]
    return {"message": "Item deleted successfully"}
```

---

## Testing Your CRUD Application

After defining your endpoints, test your CRUD API using FastAPI's automatic documentation (accessible at `/docs`) or tools like Postman and curl.

---

## Lesson Assignment

Extend the CRUD application by adding another resource, such as `User`, with its own set of CRUD endpoints. Consider implementing additional features like pagination for the read operation or validation checks for the create and update operations.

---

## Additional Resources

- [FastAPI Official Documentation](https://fastapi.tiangolo.com/tutorial/)
- [Pydantic Documentation](https://pydantic-docs.helpmanual.io/)

---
