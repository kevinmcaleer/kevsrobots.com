---
title: Dashboard and Monitoring
description: Learn how to deploy the Kubernetes Dashboard, access metrics, and monitor your K3s cluster using lightweight tools.
layout: lesson
type: page
cover: assets/k3s02.jpg
date_updated: 2025-05-24
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
