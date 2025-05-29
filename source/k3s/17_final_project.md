---
title: Final Project – Deploying a Multi-Service App
description: Bring together everything you've learned to deploy a real, multi-service application on your K3s cluster with persistence, ingress, and CI/CD integration.
layout: lesson
type: page
cover: assets/k3s04.jpg
date_updated: 2025-05-24
---

![Cover]({{page.cover}}){:class="cover"}

---

🎉 You've made it to the final project!

Now it’s time to apply everything you’ve learned and build a **real Kubernetes deployment** on your Raspberry Pi cluster using K3s.

---

## 🎯 Objective

Deploy a complete multi-service web application with:

- A **frontend** (e.g., Nginx or Flask)
- A **backend API** (e.g., FastAPI or Node.js)
- A **Redis** or **PostgreSQL** database
- Persistent storage
- Ingress routing with TLS
- Optional: CI/CD integration via GitHub Actions or ArgoCD

---

## 📁 Project Structure Example

Your repo should include:

```text

k8s/
├── frontend-deployment.yaml
├── backend-deployment.yaml
├── database-deployment.yaml
├── services.yaml
├── ingress.yaml
├── pvc.yaml
└── secrets.yaml

```

---

## ✅ Requirements

- All services deployed with Kubernetes **Deployments**
- Internal Services defined with **ClusterIP**
- One or more **PersistentVolumeClaims** for data
- Public access via **Ingress**
- Optionally: Helm chart or GitOps integration

---

## 🛠 Steps to Complete

### 1. Choose Your Stack

You can build your own or fork an existing sample, such as:

- Flask + Redis
- React + FastAPI + PostgreSQL
- WordPress + MariaDB (via Helm)

---

### 2. Write Deployment and Service YAMLs

Example:

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: api
spec:
  replicas: 1
  template:
    spec:
      containers:
        - name: api
          image: kevdev/my-api:latest
          ports:
            - containerPort: 8000
---
apiVersion: v1
kind: Service
metadata:
  name: api
spec:
  ports:
    - port: 80
      targetPort: 8000
  selector:
    app: api
```

---

### 3. Set Up Storage and Secrets

- Create a PVC for databases
- Store credentials using `kubectl create secret generic ...`

---

### 4. Configure Ingress and TLS

Use a subdomain like `app.local` with an `Ingress` rule and test via `/etc/hosts`.

---

### 5. Automate Deployment

- Use `kubectl apply -f k8s/` via GitHub Actions
- Or sync a Git repo with ArgoCD or Flux

---

## 🧪 Bonus Challenges

- Enable horizontal pod autoscaling
- Add Prometheus + Grafana monitoring
- Use a custom Helm chart to package your app
- Deploy to a second K3s cluster (multi-cluster practice)

---

## 📸 Share Your Work

- Take a photo of your Raspberry Pi cluster
- Share your architecture diagram or GitHub repo
- Post your build on GitHub, Mastodon, or Reddit with the tag `#k3sOnPi`

---

## 🏁 Summary

This final project ties together:

- Node setup
- K3s cluster installation
- Deployments, services, storage, ingress, and GitOps

You've built a full-featured Kubernetes platform on affordable hardware — congrats! 🎉

---

Next up: [Course Summary](18_summary)

---
