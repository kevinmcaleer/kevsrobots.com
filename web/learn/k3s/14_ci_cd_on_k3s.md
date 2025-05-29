---
layout: lesson
title: CI/CD on K3s
author: Kevin McAleer
type: page
cover: assets/k3s01.jpg
date: 2025-06-01
previous: 13_using_helm.html
next: 15_custom_cni.html
description: Learn how to set up continuous integration and deployment workflows for
  your K3s cluster using GitHub Actions and GitOps tools like ArgoCD or Flux.
percent: 70
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

Running apps manually is fine for testing, but to manage real applications, you need **CI/CD** — Continuous Integration and Continuous Deployment.

In this lesson, you’ll learn how to automate deployment to your K3s cluster using tools like **GitHub Actions**, **kubectl**, and **GitOps platforms** such as **ArgoCD** or **Flux**.

---

## 🔄 CI/CD Overview

A typical CI/CD pipeline for K3s:

1. Push to GitHub repo
2. GitHub Actions builds container image
3. Push image to Docker Hub or GHCR
4. Deploy to K3s using:
   - `kubectl apply`
   - or GitOps controller (ArgoCD/Flux)

---

## 🛠 Option 1: GitHub Actions + kubectl

### 🧱 Step 1: Set Up Kubeconfig Access

To allow GitHub Actions to deploy, save your `~/.kube/config` from your master node, base64-encode it:

```bash
cat ~/.kube/config | base64 -w 0
````

Add it as a **GitHub Secret** (e.g., `KUBECONFIG_B64`).

---

### 📄 Step 2: Sample GitHub Actions Workflow

```yaml
name: Deploy to K3s

on:
  push:
    branches:
      - main

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - name: Checkout code
        uses: actions/checkout@v3

      - name: Set up kubectl
        run: |
          echo "${{ secrets.KUBECONFIG_B64 }}" | base64 -d > kubeconfig
          export KUBECONFIG=$PWD/kubeconfig
          kubectl version --client

      - name: Apply manifests
        run: |
          kubectl apply -f k8s/
```

Your repo should have a `k8s/` folder with Kubernetes manifests.

---

## 🚀 Option 2: GitOps with ArgoCD or Flux

GitOps tools automatically sync your cluster with a Git repository.

### 📦 Benefits:

- Full version control of deployments
- Rollbacks via Git history
- Auto-syncing of environments

### 🧰 Getting Started with ArgoCD:

```bash
kubectl create namespace argocd
kubectl apply -n argocd \
  -f https://raw.githubusercontent.com/argoproj/argo-cd/stable/manifests/install.yaml
```

Access ArgoCD UI:

```bash
kubectl port-forward svc/argocd-server -n argocd 8080:443
```

Default login:

- user: `admin`
- password: (from secret)

```bash
kubectl get secret argocd-initial-admin-secret -n argocd -o jsonpath="{.data.password}" | base64 -d
```

Connect your Git repo and ArgoCD will deploy automatically on changes.

---

## 📋 Summary

You now know how to:

- Use GitHub Actions to deploy apps to K3s
- Securely handle kubeconfig in CI pipelines
- Use ArgoCD or Flux for GitOps-style automation
- Build production-ready workflows from your Raspberry Pi cluster

---

Next up: [Advanced Networking & CNI](15_custom_cni)

---
