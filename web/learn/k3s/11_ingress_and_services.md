---
layout: lesson
title: Ingress and Services
author: Kevin McAleer
type: page
cover: assets/k3s04.jpg
date: 2025-06-01
previous: 10_storage_and_volumes.html
next: 12_deploying_apps.html
description: Learn how to expose Kubernetes applications to your network using Services
  and Ingress controllers in K3s.
percent: 55
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

By default, Kubernetes services are only accessible inside the cluster. To make apps available on your local network or the internet, you'll need to use **Services** and optionally an **Ingress controller**.

K3s includes **Traefik** as its default ingress controller, which makes this easy to configure.

---

## 🌐 What Is a Service?

A Kubernetes **Service** is an abstraction that exposes a set of pods to other services or users.

| Type        | Use Case                         |
|-------------|----------------------------------|
| ClusterIP   | Internal-only (default)          |
| NodePort    | Exposes app on a fixed port      |
| LoadBalancer| Cloud environments (external IP) |
{:class="table table-striped"}

---

## 🚀 Exposing a Pod with a Service

Let’s say you have a deployment called `hello-world` (like from Lesson 8). You can expose it:

```bash
kubectl expose deployment hello-world --port=80 --type=NodePort
````

Check the assigned port:

```bash
kubectl get svc hello-world
```

Then open `http://<node-ip>:<nodePort>` in your browser.

---

## 🌍 What Is an Ingress?

Ingress is a more flexible way to expose multiple services using a **reverse proxy** and **host/path routing**.

Example:

* `http://pi.local/app1` → nginx app
* `http://pi.local/app2` → another service

K3s ships with **Traefik**, a built-in Ingress controller.

---

## 🧱 Step 1: Define an Ingress Resource

Create a file called `hello-ingress.yaml`:

```yaml
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: hello-ingress
  annotations:
    traefik.ingress.kubernetes.io/router.entrypoints: web
spec:
  rules:
  - host: hello.local
    http:
      paths:
      - path: /
        pathType: Prefix
        backend:
          service:
            name: hello-world
            port:
              number: 80
```

Apply it:

```bash
kubectl apply -f hello-ingress.yaml
```

---

## 🛠 Step 2: Update Your Local DNS

On your workstation, edit `/etc/hosts`:

```plaintext
192.168.1.100 hello.local
```

Replace `192.168.1.100` with your **master node IP**.

Now try visiting:

```plaintext
http://hello.local
```

---

## 🧪 Verify Ingress

Check if the Ingress was created:

```bash
kubectl get ingress
```

Check Traefik logs (optional):

```bash
kubectl logs -n kube-system -l app=traefik
```

---

## 🔒 Optional: Enable HTTPS with Let’s Encrypt

K3s and Traefik support automated HTTPS with Let's Encrypt — but you’ll need a real domain and port 80/443 open.

If you'd like to add this, let me know and I can walk you through it.

---

## ✅ Summary

You now know how to:

* Use `NodePort` to expose services to the LAN
* Create an Ingress resource with path/host routing
* Use Traefik, the default Ingress controller in K3s
* Set up basic DNS for friendly URLs

---

Next up: [Deploying Apps](12_deploying_apps)

---
