---
layout: lesson
title: Securing Your Cluster
author: Kevin McAleer
type: page
cover: assets/k3s03.jpg
date: 2025-06-01
previous: 15_custom_cni.html
next: 17_final_project.html
description: Learn how to secure your K3s cluster with role-based access control (RBAC),
  secrets management, TLS, and best practices for running in a trusted environment.
percent: 80
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

Kubernetes is powerful — but it’s also complex and needs to be properly secured, even in small or home lab clusters like K3s on Raspberry Pi.

This lesson will walk you through essential security best practices, including **RBAC**, **secrets**, **TLS**, and **attack surface reduction**.

---

## 🛡️ 1. Role-Based Access Control (RBAC)

RBAC controls **who can do what** in your cluster.

### 📄 Example: Read-Only User

Create a read-only role and binding:

```yaml
apiVersion: rbac.authorization.k8s.io/v1
kind: Role
metadata:
  namespace: default
  name: read-only
rules:
- apiGroups: [""]
  resources: ["pods", "services"]
  verbs: ["get", "list"]
---
apiVersion: rbac.authorization.k8s.io/v1
kind: RoleBinding
metadata:
  name: read-only-binding
  namespace: default
subjects:
- kind: User
  name: devuser
  apiGroup: rbac.authorization.k8s.io
roleRef:
  kind: Role
  name: read-only
  apiGroup: rbac.authorization.k8s.io
````

Apply it with `kubectl apply -f`.

> 🔐 Use service accounts and OIDC for tighter control.

---

## 🔑 2. Secrets Management

Use `Secret` resources to manage credentials, API tokens, or sensitive configs.

```yaml
apiVersion: v1
kind: Secret
metadata:
  name: my-secret
type: Opaque
data:
  username: dXNlcg==
  password: c2VjdXJlcGFzcw==
```

To create secrets from literal values:

```bash
kubectl create secret generic my-secret \
  --from-literal=username=user \
  --from-literal=password=securepass
```

> Secrets are **base64-encoded**, not encrypted. For stronger security, use **sealed-secrets** or **Vault**.

---

## 🔐 3. Enable TLS (with Ingress)

If you use Traefik or another ingress controller, enable TLS for encrypted connections:

* Use **Let’s Encrypt** for public domains
* Use **self-signed certs** or an internal CA for private clusters

Example with cert-manager and Traefik:

```bash
kubectl apply -f https://github.com/jetstack/cert-manager/releases/download/v1.13.0/cert-manager.yaml
```

Then create an `Ingress` resource with `tls:` and a `Certificate` resource.

---

## 🧱 4. Secure the K3s API

### Best practices:

* Don’t expose `6443` to the internet
* Use **firewall rules** to allow only internal access
* Remove unused components using `--disable` flags during K3s install:

  ```bash
  INSTALL_K3S_EXEC="--disable servicelb --disable traefik --disable metrics-server"
  ```

---

## 🔍 5. Audit & Monitoring

* Enable **audit logging** in K3s
* Use **Falco** or **Sysdig** for runtime security monitoring
* Use `kubectl auth can-i` to check RBAC rules:

  ```bash
  kubectl auth can-i create deployments --as devuser
  ```

---

## 🧠 Bonus Tips

| Task                     | Recommendation                             |
| ------------------------ | ------------------------------------------ |
| Avoid running as root    | Use non-root containers                    |
| Use namespaces           | Isolate environments                       |
| Keep kubeconfig safe     | Restrict permissions and store it securely |
| Regularly update images  | Especially public base images              |
| Restrict access to nodes | Use SSH keys and firewalls                 |
{:class="table table-striped"}

---

## ✅ Summary

You now know how to:

* Use RBAC to manage user permissions
* Store sensitive data using Kubernetes Secrets
* Protect traffic with TLS via Ingress
* Harden your K3s setup against unauthorized access

---

Next up: [Final Project](17_final_project)

---
