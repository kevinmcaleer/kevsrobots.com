---
title: Deploying Your FastAPI & Pydantic App
description: >-
    Learn how to deploy your FastAPI application, utilizing Pydantic for data validation, to Heroku for production use.
layout: lesson
type: page
cover: assets/10.png
---

![Deploying FastAPI and Pydantic Application]({{ page.cover }}){:class="cover"}

## Preparing for Deployment

Deploying a FastAPI application involves several key steps to ensure your application is ready for production. This includes setting up a production server, ensuring your application is secure, and choosing the right hosting platform.

---

## Requirements for Deployment

Before deploying, ensure your application:

- Is using a production-ready server like Uvicorn or Gunicorn.
- Has environment variables configured for any sensitive data.
- Includes a `requirements.txt` file with all the necessary Python packages.

---

## Deploying to Heroku

Heroku is a cloud platform that simplifies the deployment process. Here's a basic guide to deploying your FastAPI and Pydantic application to Heroku.

---

### Step 1: Prepare Your Application

Make sure your application structure is ready for deployment. This includes organizing your application files, including a `Procfile`, which tells Heroku how to run your application.

---

#### Procfile

```plaintext
web: uvicorn main:app --host=0.0.0.0 --port=${PORT:-5000}
```

This `Procfile` instructs Heroku to start your FastAPI application with Uvicorn, binding to the port assigned by Heroku.

---

### Step 2: Set Up Heroku

If you haven't already, install the Heroku CLI and create a new Heroku app:

```bash
heroku login
heroku create your-app-name
```

---

### Step 3: Deploy Your Application

Commit your code to a Git repository if you haven't done so. Then, deploy your application to Heroku using Git:

```bash
git init
heroku git:remote -a your-app-name
git add .
git commit -am "Initial deploy"
git push heroku master
```

---

### Step 4: Verify the Deployment

Once deployed, Heroku will provide a URL to access your application. Visit this URL to ensure your FastAPI application is running correctly.

---

## Post-Deployment Considerations

After deploying, monitor your application's performance and be prepared to scale your Heroku dynos according to the load. Additionally, consider setting up a continuous deployment pipeline for easier updates.

---

## Lesson Assignment

Deploy a simple FastAPI application that includes Pydantic models for data validation to Heroku. Ensure your application is accessible via a public URL and test its endpoints to confirm they are functioning as expected in the production environment.

---

## Additional Resources

- [Heroku Documentation](https://devcenter.heroku.com/)
- [FastAPI Deployment](https://fastapi.tiangolo.com/deployment/)
- [Uvicorn Deployment](https://www.uvicorn.org/deployment/)

---
