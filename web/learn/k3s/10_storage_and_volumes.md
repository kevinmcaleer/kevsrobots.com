---
layout: lesson
title: Storage and Volumes
author: Kevin McAleer
type: page
cover: assets/k3s03.jpg
date: 2025-06-01
previous: 09_dashboard_and_monitoring.html
next: 11_ingress_and_services.html
description: Learn how storage works in Kubernetes, how to use persistent volumes
  in K3s, and set up storage that survives pod restarts.
percent: 50
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

In Kubernetes, storage isn’t tied to a container like in Docker — it’s managed independently as **persistent volumes (PVs)** and **persistent volume claims (PVCs)**.

This lesson will show you how **storage works in K3s**, how to use the **built-in local-path provisioner**, and how to prepare for **external storage options** (like NFS or USB SSDs).

---

## 💾 How Storage Works in K3s

K3s includes a **default dynamic storage provisioner** called `local-path-provisioner`.

- It uses local disk storage on the node running the pod
- PVCs are automatically provisioned
- Easy to use, but not shared across nodes

> ⚠️ If a pod moves to a different node, its data may not follow.

---

## 📁 Step 1: Test with a Simple PVC

Create a file `pvc-demo.yaml`:

```yaml
apiVersion: v1
kind: PersistentVolumeClaim
metadata:
  name: demo-pvc
spec:
  accessModes:
    - ReadWriteOnce
  resources:
    requests:
      storage: 1Gi
````

Apply it:

```bash
kubectl apply -f pvc-demo.yaml
```

---

## 📦 Step 2: Mount PVC in a Pod

Create a pod with a volume mount:

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: pvc-tester
spec:
  containers:
    - name: busybox
      image: busybox
      command: [ "sleep", "3600" ]
      volumeMounts:
        - mountPath: "/data"
          name: myvolume
  volumes:
    - name: myvolume
      persistentVolumeClaim:
        claimName: demo-pvc
```

Apply it:

```bash
kubectl apply -f pvc-tester.yaml
```

Then exec into the pod and test:

```bash
kubectl exec -it pvc-tester -- sh
# Inside pod:
echo "Hello Pi Cluster" > /data/test.txt
cat /data/test.txt
```

---

## 🧠 Where Is the Data Stored?

On the node hosting the pod, the data is typically under:

```bash
/opt/local-path-provisioner
```

Each PVC is stored in its own directory.

---

## 📤 External Storage Options (Optional)

For more advanced setups:

| Option     | Benefit                        | Tool                                  |
| ---------- | ------------------------------ | ------------------------------------- |
| USB SSD    | Fast, local storage            | Mount manually or with Longhorn       |
| NFS Server | Shared across all nodes        | Use `nfs-subdir-external-provisioner` |
| Ceph       | Highly available block storage | More complex to set up                |
{:class="table table-striped"}

> 🛠 Consider using **Longhorn** if you want persistent block storage with replication and snapshots.

---

## 🧪 Cleanup

To remove test volumes and pods:

```bash
kubectl delete pod pvc-tester
kubectl delete pvc demo-pvc
```

---

## ✅ Summary

You now know how to:

- Create persistent volume claims in K3s
- Use local-path provisioner for basic storage needs
- Mount volumes into pods
- Plan for external and scalable storage options

---

Next up: [Ingress and Services](11_ingress_and_services)

---
