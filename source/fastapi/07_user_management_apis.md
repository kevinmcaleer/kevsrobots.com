---
title: Securing Endpoints with JWT Tokens in FastAPI
description: Learn how to use JWT tokens for securing API endpoints, ensuring that only authenticated users can access specific functionalities in your FastAPI application.
layout: lesson
type: page
cover: assets/7.png
---

![Securing Endpoints with JWT Tokens]({{ page.cover }}){:class="cover"}

## Introduction

After implementing registration and login functionalities, the next step is to secure your API endpoints. By requiring a valid JWT token for access, you can ensure that only authenticated users can perform certain actions.

---

## Understanding JWT Tokens

JWT tokens are a secure way to transmit information between parties as a JSON object. In the context of authentication, they are used to verify that the person making a request to your API is indeed who they claim to be.

---

## Securing an Endpoint

To secure an endpoint, we'll use FastAPI's dependency injection system to create a dependency that extracts and verifies the JWT token from the request headers.

---

### Creating a Token Dependency

First, we need a function that will extract the token from the request, decode it, and verify its validity.

```python
from fastapi import HTTPException, Depends, Security
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from jose import jwt, JWTError
from sqlalchemy.orm import Session
from .database import get_db
from . import models

security = HTTPBearer()

def get_current_user(token: HTTPAuthorizationCredentials = Security(security), db: Session = Depends(get_db)):
    try:
        payload = jwt.decode(token.credentials, SECRET_KEY, algorithms=[ALGORITHM])
        email: str = payload.get("sub")
        if email is None:
            raise HTTPException(status_code=401, detail="Invalid authentication credentials")
        user = db.query(models.User).filter(models.User.email == email).first()
        if user is None:
            raise HTTPException(status_code=401, detail="User not found")
        return user
    except JWTError:
        raise HTTPException(status_code=401, detail="Invalid token")
```

---

### Applying the Dependency to an Endpoint

Now, let's use our `get_current_user` dependency to secure an endpoint. This example shows how to create an endpoint that returns user profile information only if the user is authenticated.

```python
@app.get("/users/me/")
def read_user_me(current_user: models.User = Depends(get_current_user)):
    return current_user
```

---

## Revoking JWT Tokens

It's important to note that JWT tokens cannot be "revoked" like traditional session tokens since they are stateless. However, you can implement token expiry or use a server-side blacklist for tokens that should no longer be valid.

---

## Summary

You've learned how to secure API endpoints using JWT tokens, an essential aspect of building secure web applications. This method allows you to control access to your API, ensuring that only authenticated users can access sensitive information or perform certain actions.

---

## Additional Resources

- [FastAPI - Security](https://fastapi.tiangolo.com/tutorial/security/)
- [PyJWT - JSON Web Token library in Python](https://pyjwt.readthedocs.io/en/latest/)
- [FastAPI Security - Advanced Usage](https://fastapi.tiangolo.com/advanced/security/)

---

## Lesson Assignment

Experiment with securing another endpoint in your application, perhaps one that modifies data. Reflect on how the security requirements might differ between read and write operations, and how you can use JWT tokens to enforce these requirements.

---
