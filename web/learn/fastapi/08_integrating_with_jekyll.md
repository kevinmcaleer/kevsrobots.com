---
layout: lesson
title: Integrating FastAPI Authentication with a Jekyll Static Site
author: Kevin McAleer
type: page
cover: assets/8.png
date: 2024-03-28
previous: 07_user_management_apis.html
next: 09_deploying_the_solution.html
description: Learn how to integrate your FastAPI authentication system with a Jekyll
  static site, enabling dynamic content delivery based on user authentication.
percent: 88
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


![Integrating FastAPI with Jekyll]({{ page.cover }}){:class="cover"}

## Introduction

Integrating a FastAPI backend with a Jekyll static site allows you to combine the speed and security of static content with the flexibility and power of dynamic content generation and user authentication.

---

## Overview of Integration

The integration process involves:
1. Creating API endpoints in FastAPI that serve dynamic content.
2. Fetching this content from your Jekyll site using JavaScript based on the user's authentication status.

---

## Creating a Dynamic Content Endpoint in FastAPI

First, ensure you have a secure endpoint that returns dynamic content. This content could be user-specific data or any content that changes based on the user's state.

```python
from fastapi import FastAPI, Depends

app = FastAPI()

@app.get("/dynamic/")
def dynamic_content(current_user: models.User = Depends(get_current_user)):
    # Logic to return dynamic content
    return {"message": f"Hello, {current_user.username}! This content is dynamic."}
```

---

## Fetching Dynamic Content in Jekyll

In your Jekyll site, you can use JavaScript to fetch the dynamic content from FastAPI. Here's a simple example using the Fetch API:

```html
<script>
document.addEventListener("DOMContentLoaded", function() {
  fetch('http://localhost:8000/dynamic/', {
    method: 'GET',
    headers: {
      'Authorization': `Bearer ${localStorage.getItem('token')}`
    }
  })
  .then(response => response.json())
  .then(data => {
    document.getElementById('dynamic-content').innerText = data.message;
  })
  .catch(error => console.error('Error fetching dynamic content:', error));
});
</script>
<div id="dynamic-content">Loading dynamic content...</div>
```

This script should be included in the relevant HTML file of your Jekyll site. It attempts to fetch dynamic content from the FastAPI server and displays it on the page. Make sure to replace `http://localhost:8000/dynamic/` with the actual URL of your FastAPI application.

---

## Handling Authentication in Jekyll

You'll need to manage user authentication tokens in the browser. Typically, this involves storing the JWT token in `localStorage` or `sessionStorage` after login and then including it in the authorization header of your API requests.

---

## Summary

Integrating FastAPI with a Jekyll static site enables you to deliver dynamic, authenticated content alongside your static content, offering a richer user experience. This lesson covered creating a dynamic content endpoint in FastAPI and fetching this content from Jekyll, paving the way for a seamless integration.

---

## Additional Resources

- [FastAPI Official Documentation](https://fastapi.tiangolo.com/)
- [Jekyll Documentation](https://jekyllrb.com/docs/)
- [Using Fetch - Web APIs](https://developer.mozilla.org/en-US/docs/Web/API/Fetch_API/Using_Fetch)

---

## Lesson Assignment

Explore additional ways to enhance user interaction on your Jekyll site using dynamic content from FastAPI. Consider implementing a feature that displays content based on the user's profile or preferences.

---
