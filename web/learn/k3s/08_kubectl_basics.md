---
layout: lesson
title: Kubectl Basics
author: Kevin McAleer
type: page
cover: assets/k3s01.jpg
date: 2025-06-01
previous: 07_troubleshooting_install.html
next: 09_dashboard_and_monitoring.html
description: Learn the essential `kubectl` commands to interact with your K3s Kubernetes
  cluster from the command line.
percent: 40
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

With your K3s cluster up and running, it’s time to **interact with it using `kubectl`** — the command-line tool for managing Kubernetes.

This lesson will walk you through the basics of exploring and managing your cluster.

---

## ✅ Setting Up `kubectl` Access

On the **master node**, or from your development machine:

```bash
export KUBECONFIG=/etc/rancher/k3s/k3s.yaml
````

Or copy the config to your local system:

```bash
scp pi@pi-master:/etc/rancher/k3s/k3s.yaml ~/.kube/config
```

Replace `pi@pi-master` with your actual master node address.

Ensure correct file permissions:

```bash
chmod 600 ~/.kube/config
```

---

## 🔍 Inspecting the Cluster

List all nodes:

```bash
kubectl get nodes
```

List all running pods (across namespaces):

```bash
kubectl get pods -A
```

Check system components (K3s uses `kube-system` namespace):

```bash
kubectl get pods -n kube-system
```

---

## 🚀 Deploying a Simple App

Create a simple deployment:

```bash
kubectl create deployment hello-world --image=nginx
```

Expose it as a service:

```bash
kubectl expose deployment hello-world --port=80 --type=NodePort
```

Find the assigned NodePort:

```bash
kubectl get svc hello-world
```

Access it via any node's IP and the port listed.

---

## 📁 Useful Commands

| Command                          | Description                                |
| -------------------------------- | ------------------------------------------ |
| `kubectl get pods`               | List all pods in the current namespace     |
| `kubectl describe pod <name>`    | Show detailed info about a pod             |
| `kubectl logs <pod>`             | View logs from a pod                       |
| `kubectl exec -it <pod> -- bash` | Open a shell inside a pod (if it has bash) |
| `kubectl delete pod <name>`      | Delete a pod                               |
| `kubectl apply -f <file.yaml>`   | Apply resources from a YAML file           |
{:class="table table-striped"}

---

## 🧠 Pro Tips

* Use `-A` to view resources across all namespaces
* Add bash completion for `kubectl`:

  ```bash
  source <(kubectl completion bash)
  ```
  
* Use `kubectl get all` to quickly see services, pods, deployments

---

## 🔍 Example: Get Details on Your Node

```bash
kubectl describe node pi-master
```

Look at:

* Capacity
* Allocated resources
* Kubelet version
* Taints and labels

---

## ✅ Summary

You now know how to:

* Access your K3s cluster with `kubectl`
* Deploy, expose, and interact with workloads
* Use key commands to inspect cluster health

---

Next up: [Dashboard and Monitoring](09_dashboard_and_monitoring)

---
