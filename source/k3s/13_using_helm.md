---
title: Using Helm
description: Learn how to use Helm to deploy and manage applications on your K3s cluster with reusable charts and versioned releases.
layout: lesson
type: page
cover: assets/k3s06.jpg
date_updated: 2025-05-24
---

![Cover]({{page.cover}}){:class="cover"}

---

As your Kubernetes setup grows, writing and maintaining raw YAML files becomes harder to manage. That’s where **Helm** comes in.

Helm is a **package manager for Kubernetes** — like apt for Ubuntu or pip for Python. It allows you to deploy applications as **charts**, with versioning, values, and easy upgrades.

---

## 🎯 Why Use Helm?

| Feature       | Benefit                      |
|---------------|------------------------------|
| Charts        | Reusable templates for apps  |
| Versioning    | Rollbacks and upgrades       |
| `values.yaml` | Customization made easy      |
| Ecosystem     | Thousands of prebuilt charts |
{:class="table table-striped"}

---

## 🛠 Step 1: Install Helm

On your **development machine** or master node:

```bash
curl https://raw.githubusercontent.com/helm/helm/main/scripts/get-helm-3 | bash
````

Verify:

```bash
helm version
```

---

## 📦 Step 2: Add a Helm Repository

Helm charts are hosted in repositories like Docker images.

Add the Bitnami repo (for common apps):

```bash
helm repo add bitnami https://charts.bitnami.com/bitnami
helm repo update
```

---

## 🚀 Step 3: Install an App with Helm

Let’s install **WordPress** as an example:

```bash
helm install my-blog bitnami/wordpress
```

This will:

* Deploy WordPress + MariaDB
* Set up storage
* Create services

Check resources:

```bash
kubectl get all
```

---

## ⚙️ Step 4: Customize the Install

You can pass custom values inline:

```bash
helm install my-blog bitnami/wordpress \
  --set wordpressUsername=admin \
  --set wordpressPassword=secretpass
```

Or create a `values.yaml` file for reproducible installs.

---

## 🔄 Step 5: Upgrade and Rollback

Update your chart:

```bash
helm upgrade my-blog bitnami/wordpress --set wordpressBlogName="New Blog"
```

Roll back to the previous version:

```bash
helm rollback my-blog
```

List releases:

```bash
helm list
```

---

## 🧹 Uninstall a Release

```bash
helm uninstall my-blog
```

All resources created by the chart will be removed.

---

## 🧠 Common Use Cases for Helm

* Deploying **Dashboards**, **Monitoring** (e.g. Prometheus, Grafana)
* Managing **Databases**, **Message Queues**
* Packaging **your own apps** with custom charts
* Automating with **CI/CD pipelines**

---

## ✅ Summary

You now know how to:

* Install and configure Helm
* Use Helm charts to deploy apps
* Customize installs with `values.yaml`
* Upgrade, rollback, and uninstall releases

Helm makes managing Kubernetes apps much easier — especially for repeatable deployments and automation.

---

Next up: [CI/CD on K3s](14_ci_cd_on_k3s)

---
