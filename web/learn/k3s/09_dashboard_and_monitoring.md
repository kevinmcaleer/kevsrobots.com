---
layout: lesson
title: Dashboard and Monitoring
author: Kevin McAleer
type: page
cover: assets/k3s02.jpg
date: 2025-06-01
previous: 08_kubectl_basics.html
next: 10_storage_and_volumes.html
description: Learn how to deploy the Kubernetes Dashboard, access metrics, and monitor
  your K3s cluster using lightweight tools.
percent: 45
duration: 2
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

A running K3s cluster is great — but to manage it effectively, you’ll want **visibility** into its health and performance.

This lesson covers how to deploy the **Kubernetes Dashboard**, add basic **metrics monitoring**, and explore other lightweight observability tools that work well on Raspberry Pi.

---

## 🧭 Option 1: Kubernetes Dashboard

### 🔹 Step 1: Deploy the Dashboard

Apply the official YAML:

```bash
kubectl apply -f https://raw.githubusercontent.com/kubernetes/dashboard/v2.7.0/aio/deploy/recommended.yaml
````

This deploys:

* The Dashboard UI
* Metrics scrapers
* Required roles

---

### 🔹 Step 2: Create Admin User

Create a file `dashboard-admin.yaml`:

```yaml
apiVersion: v1
kind: ServiceAccount
metadata:
  name: admin-user
  namespace: kubernetes-dashboard
---
apiVersion: rbac.authorization.k8s.io/v1
kind: ClusterRoleBinding
metadata:
  name: admin-user
roleRef:
  apiGroup: rbac.authorization.k8s.io
  kind: ClusterRole
  name: cluster-admin
subjects:
- kind: ServiceAccount
  name: admin-user
  namespace: kubernetes-dashboard
```

Apply it:

```bash
kubectl apply -f dashboard-admin.yaml
```

---

### 🔹 Step 3: Access the Dashboard

Start the proxy:

```bash
kubectl proxy
```

Then open:

```text
http://localhost:8001/api/v1/namespaces/kubernetes-dashboard/services/https:kubernetes-dashboard:/proxy/
```

To get your token:

```bash
kubectl -n kubernetes-dashboard create token admin-user
```

Paste the token into the login screen.

---

## 📊 Option 2: Metrics Server (Optional for K3s)

K3s includes a built-in lightweight metrics server, but if not available:

```bash
kubectl apply -f https://github.com/kubernetes-sigs/metrics-server/releases/latest/download/components.yaml
```

Check node metrics:

```bash
kubectl top nodes
kubectl top pods
```

---

## 🧩 Other Lightweight Monitoring Tools

| Tool                     | Description                           |
| ------------------------ | ------------------------------------- |
| **KubeView**             | Visualize pods, nodes, and namespaces |
| **k9s**                  | Terminal UI for Kubernetes            |
| **Grafana + Prometheus** | Full monitoring stack (heavier)       |
| **Lens**                 | Powerful GUI app for remote clusters  |
{:class="table table-striped"}

> 🧠 Tip: For Raspberry Pi, prioritize tools with low memory footprint.

---

## 🧪 Verifying Everything Works

* Open Dashboard and view cluster state
* Check `kubectl top nodes` for metrics
* View logs and usage graphs for a running pod

---

## ✅ Summary

You now know how to:

* Deploy and access the Kubernetes Dashboard
* Use admin tokens for secure access
* Monitor basic metrics from your cluster
* Explore other lightweight tools for deeper insights

---

Next up: [Storage and Volumes](10_storage_and_volumes)

---
