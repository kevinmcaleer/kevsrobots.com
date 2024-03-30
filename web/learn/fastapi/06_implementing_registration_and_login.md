---
layout: lesson
title: Implementing Registration and Login with FastAPI
author: Kevin McAleer
type: page
cover: assets/6.png
date: 2024-03-28
previous: 05_creating_user_models.html
next: 07_user_management_apis.html
description: This lesson covers how to build registration and login endpoints in your
  FastAPI application, including password hashing and token generation.
percent: 66
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


![Implementing Registration and Login]({{ page.cover }}){:class="cover"}

## Introduction

With our user models in place, we can now implement the registration and login functionalities. This process involves securely handling passwords, verifying user credentials, and generating authentication tokens.

---

## User Registration

### Handling Passwords Securely

Before saving a user's password to the database, we must hash it. Using a library like `passlib`, we can securely hash passwords.

```python
from passlib.context import CryptContext

pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

def get_password_hash(password):
    return pwd_context.hash(password)
```

---

### Creating the Registration Endpoint

Our registration endpoint will accept username, email, and password, validate the data, hash the password, and then save the new user to the database.

```python
from fastapi import FastAPI, HTTPException, Depends
from sqlalchemy.orm import Session
from . import models, schemas
from .database import SessionLocal, engine

app = FastAPI()

# Dependency
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

@app.post("/register/")
def register_user(user: schemas.UserCreate, db: Session = Depends(get_db)):
    db_user = db.query(models.User).filter(models.User.email == user.email).first()
    if db_user:
        raise HTTPException(status_code=400, detail="Email already registered")
    hashed_password = get_password_hash(user.password)
    db_user = models.User(username=user.username, email=user.email, hashed_password=hashed_password)
    db.add(db_user)
    db.commit()
    db.refresh(db_user)
    return {"username": db_user.username, "email": db_user.email}
```

---

## User Login

### Verifying Credentials

To log in a user, we need to verify their email and password. This involves fetching the user from the database and checking the password hash.

```python
def verify_password(plain_password, hashed_password):
    return pwd_context.verify(plain_password, hashed_password)

def authenticate_user(email: str, password: str, db: Session = Depends(get_db)):
    user = db.query(models.User).filter(models.User.email == email).first()
    if not user or not verify_password(password, user.hashed_password):
        return False
    return user
```

---

### Generating JWT Tokens

After verifying the user, we generate a JWT token for session management. This token is sent back to the user and used in subsequent requests.

```python
from datetime import datetime, timedelta
import jwt

SECRET_KEY = "your_secret_key"
ALGORITHM = "HS256"

def create_access_token(data: dict, expires_delta: timedelta = None):
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=15)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt
```

---

## Summary

You've now implemented the registration and login functionalities, including secure password handling and JWT token generation. These are key components of our user authentication system. In the next lessons, we'll explore how to use these tokens for authenticating API requests and managing user sessions.

---

## Additional Resources

- [JWT.io - JSON Web Tokens Introduction](https://jwt.io/introduction/)
- [FastAPI Security](https://fastapi.tiangolo.com/tutorial/security/)
- [Passlib](https://passlib.readthedocs.io/en/stable/)

---

## Lesson Assignment

Implement an endpoint for logging out users. Consider how you would invalidate the JWT token and what impact this has on the server and client side of your application.

---
