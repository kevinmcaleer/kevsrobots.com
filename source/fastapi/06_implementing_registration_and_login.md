---
title: Implementing Registration and Login with FastAPI
description: This lesson covers how to build registration and login endpoints in your FastAPI application, including password hashing and token generation.
layout: lesson
type: page
cover: assets/6.png
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
