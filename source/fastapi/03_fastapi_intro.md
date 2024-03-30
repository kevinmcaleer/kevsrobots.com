---
title: Introduction to FastAPI
description: Understand the core concepts of FastAPI, how it compares to other web frameworks, and its advantages in building APIs, especially for user authentication.
layout: lesson
type: page
cover: assets/3.png
---

![FastAPI Overview]({{ page.cover }}){:class="cover"}

## What is FastAPI?

FastAPI is a modern, fast (high-performance) web framework for building APIs with Python 3.7+ based on standard Python type hints. Its key features include automatic API documentation, data validation, serialization, and asynchronous request handling. FastAPI is designed to make it easy to create a web API that is fast to code, easy to understand, and ready for production.

## Core Features of FastAPI

- **Speed:** FastAPI provides very high performance, on par with NodeJS and Go, thanks to Starlette for the web parts and Pydantic for the data parts.
- **Quick Coding:** Its design allows for fast development. Define your API using Python type hints, and FastAPI does the rest.
- **Automatic Documentation:** With FastAPI, you get interactive API documentation automatically generated via Swagger UI and ReDoc.
- **Data Validation and Serialization:** Uses Pydantic for data validation, serialization, and model binding, ensuring that the data your API receives and sends is correct.
- **Asynchronous Support:** FastAPI supports asynchronous request handling, making it suitable for high-concurrency situations like handling user authentication and interaction.

## Why FastAPI for User Authentication?

Choosing FastAPI for user authentication in your project comes with several benefits:
- **Security:** FastAPI includes several built-in security features, such as support for OAuth2 with Password (and hashing), JWT tokens, and more, which are crucial for authentication systems.
- **Scalability:** The asynchronous support makes it easy to scale your authentication system for a high number of requests.
- **Developer Experience:** The automatic documentation and type hinting make developing and testing your API a breeze.

## First Steps with FastAPI

Let's create a simple FastAPI application to understand its structure:

1. **Create a Main Application File:**

In your project directory, ensure you have a file named `main.py`. This file will be the entry point for your FastAPI application.

2. **Write Your First FastAPI Code:**

Open `main.py` and add the following code:

```python
from fastapi import FastAPI

app = FastAPI()

@app.get("/")
def read_root():
    return {"Hello": "World"}
```

3. **Run Your FastAPI Application:**

Use the following command to run your application:

```bash
uvicorn main:app --reload
```

The `--reload` flag makes the server restart after code changes. This is useful during development but should be removed in production.

## Testing Your API

With your FastAPI application running, visit `http://127.0.0.1:8000` in your web browser. You should see a response from your API.

## Exploring the Automatic Documentation

FastAPI automatically generates documentation for your API. Access this documentation by visiting `http://127.0.0.1:8000/docs` for the Swagger UI or `http://127.0.0.1:8000/redoc` for ReDoc.

## Summary

You've now created your first FastAPI application and explored some of its core features. In the coming lessons, we'll dive deeper into building a user authentication system, starting with modeling our user data.

## Additional Resources

- [FastAPI Official Documentation](https://fastapi.tiangolo.com/)
- [Pydantic Documentation](https://pydantic-docs.helpmanual.io/)

## Lesson Assignment

Experiment with adding another route to your FastAPI application. Try returning different types of data, such as a list or a dictionary with nested data. Reflect on how FastAPI's automatic documentation updates with your changes.

---
