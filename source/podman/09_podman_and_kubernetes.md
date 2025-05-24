---
title: Podman and Kubernetes
description: Learn how to use Podman with Kubernetes — including generating Kubernetes YAML files, running them locally, and preparing workloads for production.
layout: lesson
type: page
cover: assets/podman03.jpg
date_updated: 2025-05-24
---

![Cover]({{page.cover}}){:class="cover"}

---

One of Podman’s most powerful features is its **native support for Kubernetes YAML**.

This makes it easy to develop container workloads locally and then **export them directly to Kubernetes** — no need to rewrite configs or convert tools.

---

## 📄 Generating Kubernetes YAML from Podman

You can create containers or pods with Podman and then export them as Kubernetes-compatible YAML.

```bash
podman generate kube mypod > mypod.yaml
````

This includes:

* Pod and container configuration
* Environment variables
* Ports and volumes
* Restart policies (as annotations)

> 🧠 The output is structured as a `Pod` definition — ready to use with `kubectl` or `podman play kube`.

---

## ▶️ Running Kubernetes YAML with Podman

Test Kubernetes manifests locally using:

```bash
podman play kube mypod.yaml
```

This will:

* Create a pod and containers from the YAML file
* Apply port mappings, environment variables, etc.
* Use Podman’s pod infrastructure

---

## 📦 Example Workflow

```bash
# Create a pod manually
podman pod create --name apppod -p 8080:80

# Add services
podman run -dt --pod apppod --name app nginx
podman run -dt --pod apppod --name db redis

# Export as Kubernetes YAML
podman generate kube apppod > apppod.yaml

# Recreate from YAML
podman play kube apppod.yaml
```

This lets you build your apps locally and **deploy using the same manifests** you’ll use in production.

---

## 🔄 Integration with Kubernetes Clusters

While Podman is not a Kubernetes runtime itself (like `CRI-O`), it plays a key role in:

* Prototyping and testing deployments
* Validating configurations before deploying to the cloud
* Generating boilerplate YAML for clusters

> 🧰 Podman works well alongside Kubernetes tools like `kubectl`, `minikube`, or `kind`.

---

## 🧪 Use Case: Local Dev + Cloud Deployment

1. Develop and test containers with Podman
2. Use `generate kube` to export configuration
3. Use `kubectl apply` to deploy to a real Kubernetes cluster

This simplifies **testing and CI/CD pipelines**, and bridges local workflows with production orchestration.

---

## ⚠️ Caveats

* `podman play kube` only supports basic `Pod`, `Deployment`, and `Service` objects
* More complex Kubernetes features (e.g., Ingress, ConfigMaps, secrets) require manual YAML editing or use of Kubernetes CLI

---

## 🧩 Podman + CRI-O

In Kubernetes environments, Podman’s sibling tool — **CRI-O** — serves as the container runtime. It uses the same container libraries and OCI standards.

> 🔗 If you’re comfortable with Podman, transitioning to **CRI-O + Kubernetes** will feel very familiar.

---

Next up: [Podman in Production](10_podman_in_production)

---
