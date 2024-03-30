---
layout: lesson
title: Deploying Your FastAPI Authentication System and Jekyll Site
author: Kevin McAleer
type: page
cover: assets/9.png
date: 2024-03-28
previous: 08_integrating_with_jekyll.html
description: Learn the best practices for deploying your FastAPI application with
  user authentication and integrating it with your Jekyll static site to production.
percent: 100
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


![Deploying FastAPI and Jekyll]({{ page.cover }}){:class="cover"}

## Introduction

Deploying your FastAPI application and Jekyll site involves several steps to ensure that your application is secure, efficient, and accessible to your audience. This lesson will guide you through the deployment process, focusing on best practices and essential considerations.

---

## Pre-Deployment Checklist

Before deploying your application, ensure that:

1. **Security Measures Are in Place:** This includes using HTTPS, securing sensitive data, and implementing proper authentication and authorization checks.
2. **Environment Variables Are Configured:** Store configuration and secrets in environment variables instead of hard-coding them in your application.
3. **Database Is Ready for Production:** Ensure that your database is properly set up, optimized, and securely accessible by your application.
4. **Static Files Are Optimized:** For Jekyll, ensure that your static files (images, CSS, JS) are minimized and optimized for faster loading.

---

## Deploying FastAPI

### Choosing a Hosting Service

Several cloud providers and hosting services support FastAPI applications, including Heroku, AWS (with Elastic Beanstalk or EC2), Google Cloud Platform, and DigitalOcean. The choice depends on your project's needs, budget, and preferred infrastructure.

---

### Dockerizing Your Application

Containerizing your FastAPI application with Docker simplifies deployment and ensures consistency across different environments. Here's a simple `Dockerfile` example:

```Dockerfile
FROM python:3.8

WORKDIR /app

COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "80"]
```

---

### Deployment Process

- Build your Docker image and push it to a container registry.
- Deploy your container to the chosen hosting service.
- Ensure that your service is configured to restart your container automatically in case of failure.

---

## Deploying Jekyll

Jekyll sites can be deployed to a variety of hosting services, including GitHub Pages, Netlify, and Vercel. These platforms often provide continuous deployment from your Git repository, making updates straightforward.

1. **Build Your Jekyll Site:** Run `jekyll build` to generate the static files for your site.
2. **Configure Deployment:** Depending on your hosting service, you may need to configure specific deployment settings or files.
3. **Continuous Deployment:** If supported, set up continuous deployment to automatically build and deploy your site when you push changes to your repository.

---

## Integrating FastAPI with Jekyll

Ensure that your Jekyll site is configured to communicate with your FastAPI application. This may involve setting API endpoint URLs as environment variables or configuring CORS settings on your FastAPI application to accept requests from your Jekyll site's domain.

---

## Summary

Deploying your FastAPI application and Jekyll site to production involves careful planning and attention to security, performance, and reliability. By following best practices and utilizing modern deployment tools and services, you can ensure that your application is ready for your users.

---

## Additional Resources

- [FastAPI Deployment](https://fastapi.tiangolo.com/deployment/)
- [Jekyll Deployment Methods](https://jekyllrb.com/docs/deployment/methods/)
- [Docker Documentation](https://docs.docker.com/)

---

## Lesson Assignment

Deploy a simple FastAPI application and integrate it with a Jekyll static site on a hosting service of your choice. Document the steps you took and any challenges you faced during the deployment process.

---
