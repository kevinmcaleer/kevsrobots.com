---
layout: lesson
title: Advanced Networking & Custom CNI
author: Kevin McAleer
type: page
cover: assets/k3s02.jpg
date: 2025-06-01
previous: 14_ci_cd_on_k3s.html
next: 16_securing_your_cluster.html
description: Explore Kubernetes Container Network Interfaces (CNI) in K3s and learn
  how to replace the default Flannel with more advanced CNIs like Calico or Cilium.
percent: 75
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

K3s ships with a default CNI plugin called **Flannel**, which provides basic pod networking. But as your cluster grows or your needs become more complex (e.g. network policies, observability), you may want to use a more powerful **Container Network Interface (CNI)** plugin.

In this lesson, you’ll learn how CNI works in K3s and how to install alternatives like **Calico** or **Cilium**.

---

## 🌐 What Is CNI?

CNI stands for **Container Network Interface**, a standard that defines how containers connect to the network.

Kubernetes uses CNI plugins to:

- Assign IP addresses to pods
- Route traffic between pods and services
- Enforce network policies
- Enable multi-cluster and overlay networking

---

## ⚙️ Default in K3s: Flannel

- Lightweight and simple
- Uses VXLAN overlay by default
- No support for network policies
- Suitable for most home lab setups

---

## 🚀 Why Replace the Default CNI?

| Need                            | Recommended CNI |
|----------------------------------|------------------|
| Enforce network security rules   | Calico          |
| Deep observability / eBPF        | Cilium          |
| Multi-cluster support            | Cilium, Calico  |
| WireGuard encryption             | Cilium          |
| BGP routing                      | Calico          |
{:class="table table-striped"}

---

## 🧼 Step 1: Disable Flannel in K3s

To disable Flannel, uninstall and reinstall K3s with:

```bash
sudo /usr/local/bin/k3s-uninstall.sh
````

Then reinstall without Flannel:

```bash
curl -sfL https://get.k3s.io | INSTALL_K3S_EXEC="--flannel-backend=none --disable-network-policy" sh -
```

---

## 🧱 Step 2: Install Calico (Example)

Apply the official Calico manifests:

```bash
kubectl apply -f https://raw.githubusercontent.com/projectcalico/calico/v3.26.1/manifests/tigera-operator.yaml
```

Then install the Calico custom resource:

```bash
kubectl apply -f https://raw.githubusercontent.com/projectcalico/calico/v3.26.1/manifests/custom-resources.yaml
```

Verify:

```bash
kubectl get pods -n calico-system
```

---

## 🧪 Test Network Policies (Optional)

Calico allows you to enforce pod-to-pod communication rules. For example:

```yaml
apiVersion: networking.k8s.io/v1
kind: NetworkPolicy
metadata:
  name: deny-all
spec:
  podSelector: {}
  policyTypes:
    - Ingress
```

This would block all incoming traffic unless explicitly allowed.

---

## ⚠️ Notes on ARM Compatibility

- Always check the **ARM64 support** for the CNI plugin you want to use.
- Both Calico and Cilium support ARM (Raspberry Pi 4/5) as of recent versions.
- Monitor memory and CPU usage — some CNIs (like Cilium with eBPF) are more demanding.

---

## ✅ Summary

You now know how to:

- Understand Kubernetes CNI and its role in networking
- Disable K3s’s default Flannel backend
- Install a custom CNI like Calico or Cilium
- Begin using network policies and advanced features

---

Next up: [Securing Your Cluster](16_securing_your_cluster)

---
