---
layout: lesson
title: Deploying Apps
author: Kevin McAleer
type: page
cover: assets/k3s05.jpg
date: 2025-06-01
previous: 11_ingress_and_services.html
next: 13_using_helm.html
description: Learn how to deploy real-world applications on your K3s cluster using
  Deployments, Services, and Ingress.
percent: 60
duration: 3
date_updated: 2025-05-24
navigation:
- name: Running K3s on Raspberry Pi
- content:
  - section: Introduction
    content:
    - name: Introduction to K3s on Raspberry Pi
      link: 01_intro.html
  - section: Preparing Your Raspberry Pi Cluster
    content:
    - name: Hardware Requirements
      link: 02_hardware_requirements.html
    - name: Installing the Operating System
      link: 03_os_installation.html
    - name: Cluster Networking
      link: 04_cluster_networking.html
  - section: Installing K3s
    content:
    - name: Installing K3s
      link: 05_installing_k3s.html
    - name: Adding Nodes
      link: 06_adding_nodes.html
    - name: Troubleshooting Installation
      link: 07_troubleshooting_install.html
  - section: Using Your K3s Cluster
    content:
    - name: Kubectl Basics
      link: 08_kubectl_basics.html
    - name: Dashboard and Monitoring
      link: 09_dashboard_and_monitoring.html
    - name: Storage and Volumes
      link: 10_storage_and_volumes.html
    - name: Ingress and Services
      link: 11_ingress_and_services.html
  - section: Real-World Deployments
    content:
    - name: Deploying Apps
      link: 12_deploying_apps.html
    - name: Using Helm
      link: 13_using_helm.html
    - name: CI/CD on K3s
      link: 14_ci_cd_on_k3s.html
  - section: Advanced Topics
    content:
    - name: Advanced Networking & Custom CNI
      link: 15_custom_cni.html
    - name: Securing Your Cluster
      link: 16_securing_your_cluster.html
  - section: Final Project
    content:
    - name: "Final Project \u2013 Deploying a Multi-Service App"
      link: 17_final_project.html
  - section: Summary
    content:
    - name: Course Summary
      link: 18_summary.html
---


![Cover]({{page.cover}}){:class="cover"}

---

Now that your K3s cluster is running and accessible via services and ingress, it's time to deploy a **real-world application**.

In this lesson, you'll deploy a simple web app with a backing Redis database using **Deployments**, **Services**, and an **Ingress** rule.

---

## 🧩 Example App: Python + Redis

We’ll deploy:

- A Flask web app (`web`)
- A Redis database (`redis`)

These will be connected using environment variables and services.

---

## 📁 Step 1: Create Deployment YAML

Create a file called `webapp.yaml`:

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: web
spec:
  replicas: 1
  selector:
    matchLabels:
      app: web
  template:
    metadata:
      labels:
        app: web
    spec:
      containers:
      - name: web
        image: kevdev/web-redis-demo:latest
        ports:
        - containerPort: 5000
        env:
        - name: REDIS_HOST
          value: redis
---
apiVersion: v1
kind: Service
metadata:
  name: web
spec:
  selector:
    app: web
  ports:
  - protocol: TCP
    port: 80
    targetPort: 5000
````

This assumes you have a Docker image like `kevdev/web-redis-demo` on Docker Hub — update it with your own image if needed.

---

## 🧱 Step 2: Add Redis Deployment

Append this to the same file or create a separate `redis.yaml`:

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: redis
spec:
  replicas: 1
  selector:
    matchLabels:
      app: redis
  template:
    metadata:
      labels:
        app: redis
    spec:
      containers:
      - name: redis
        image: redis:alpine
---
apiVersion: v1
kind: Service
metadata:
  name: redis
spec:
  selector:
    app: redis
  ports:
  - protocol: TCP
    port: 6379
```

---

## 🚀 Step 3: Apply to the Cluster

```bash
kubectl apply -f webapp.yaml
kubectl apply -f redis.yaml
```

Verify everything is running:

```bash
kubectl get pods
kubectl get svc
```

---

## 🌐 Step 4: Add Ingress

Create a file called `webapp-ingress.yaml`:

```yaml
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: web-ingress
  annotations:
    traefik.ingress.kubernetes.io/router.entrypoints: web
spec:
  rules:
  - host: web.local
    http:
      paths:
      - path: /
        pathType: Prefix
        backend:
          service:
            name: web
            port:
              number: 80
```

Apply it:

```bash
kubectl apply -f webapp-ingress.yaml
```

Add this to `/etc/hosts` on your local machine:

```text
192.168.1.100 web.local
```

Visit `http://web.local` in your browser!

---

## 🧪 Optional: Scale Your App

```bash
kubectl scale deployment web --replicas=3
```

See the load balancing in action (especially with Ingress).

---

## ✅ Summary

You now know how to:

- Deploy multi-container apps using YAML
- Connect services via internal DNS
- Expose apps using Ingress
- Scale deployments with one command

---

Next up: [Using Helm](13_using_helm)

---
