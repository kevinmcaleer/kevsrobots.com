---
title: Kubectl Basics
description: Learn the essential `kubectl` commands to interact with your K3s Kubernetes cluster from the command line.
layout: lesson
type: page
cover: assets/k3s01.jpg
date_updated: 2025-05-24
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
