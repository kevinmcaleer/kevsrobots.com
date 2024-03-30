---
layout: lesson
title: Creating User Models with Pydantic and SQLAlchemy
author: Kevin McAleer
type: page
cover: assets/5.png
date: 2024-03-28
previous: 04_user_authentication_flow.html
next: 06_implementing_registration_and_login.html
description: Learn how to define user models using Pydantic for data validation and
  SQLAlchemy for ORM, laying the foundation for our authentication system.
percent: 55
duration: 3
navigation:
- name: Building User Authentication for Static Sites with FastAPI
- content:
  - section: Introduction to User Authentication
    content:
    - name: Introduction to FastAPI and User Authentication Basics
      link: 01_course_overview.html
    - name: Setting Up the Development Environment for FastAPI
      link: 02_setting_up_the_environment.html
  - section: FastAPI and User Authentication Basics
    content:
    - name: Introduction to FastAPI
      link: 03_fastapi_intro.html
    - name: User Authentication Flow
      link: 04_user_authentication_flow.html
  - section: Building the Authentication API
    content:
    - name: Creating User Models with Pydantic and SQLAlchemy
      link: 05_creating_user_models.html
    - name: Implementing Registration and Login with FastAPI
      link: 06_implementing_registration_and_login.html
    - name: Securing Endpoints with JWT Tokens in FastAPI
      link: 07_user_management_apis.html
  - section: Integrating with a Jekyll Site
    content:
    - name: Integrating FastAPI Authentication with a Jekyll Static Site
      link: 08_integrating_with_jekyll.html
    - name: Deploying Your FastAPI Authentication System and Jekyll Site
      link: 09_deploying_the_solution.html
---


![Creating User Models]({{ page.cover }}){:class="cover"}

## Introduction

In building our authentication system, the first step is to create models that define the structure of user data. These models ensure that the data is valid, consistent, and ready for storage in a database. We'll use Pydantic for data validation and SQLAlchemy for Object-Relational Mapping (ORM) to interact with our database.

---

## Pydantic Models for Data Validation

Pydantic uses Python type annotations to validate data. This ensures that the data conforms to specified formats before we process it or save it to our database.

---

### Defining a User Pydantic Model

Let's define a simple user model with Pydantic to validate user registration data:

```python
from pydantic import BaseModel, EmailStr

class UserCreate(BaseModel):
    username: str
    email: EmailStr
    password: str
```

This model will validate that the `username` is a string, the `email` is a valid email address, and the `password` is also a string.

---

## SQLAlchemy Models for Database Interaction

SQLAlchemy is an ORM library that allows us to interact with databases using Python classes and objects. It abstracts away SQL queries, making database operations more Pythonic and secure.

---

### Defining a User SQLAlchemy Model

Now, let's define an SQLAlchemy model for our users:

```python
from sqlalchemy import Column, Integer, String
from .database import Base

class User(Base):
    __tablename__ = "users"

    id = Column(Integer, primary_key=True, index=True)
    username = Column(String, unique=True, index=True)
    email = Column(String, unique=True, index=True)
    hashed_password = Column(String)
```

Notice we're storing `hashed_password` instead of `password`. It's crucial never to store plain passwords in your database for security reasons.

---

## Integrating Pydantic and SQLAlchemy Models

While Pydantic models are great for input validation and serialization, SQLAlchemy models are used for database operations. In practice, you'll often convert between these two model types.

For example, after validating user registration data with a Pydantic model, you'll convert this data into an SQLAlchemy model before saving it to the database. Similarly, when fetching user data from the database, you'll convert SQLAlchemy models into Pydantic models before sending them to clients.

---

## Summary

You've learned how to define user models using Pydantic for validation and SQLAlchemy for database interaction. These models form the backbone of our user authentication system, ensuring data integrity and security. In the next lesson, we'll implement user registration and login functionalities using these models.

---

## Additional Resources

- [Pydantic Documentation](https://pydantic-docs.helpmanual.io/)
- [SQLAlchemy ORM Tutorial for Python Developers](https://docs.sqlalchemy.org/en/14/orm/tutorial.html)

---

## Lesson Assignment

Try adding additional fields to the `UserCreate` Pydantic model, such as `first_name` and `last_name`, and reflect on how these additions might be validated. Consider the types of validation that would be appropriate for these fields.

---
