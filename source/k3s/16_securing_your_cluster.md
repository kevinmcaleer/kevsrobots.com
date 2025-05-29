---
title: Securing Your Cluster
description: Learn how to secure your K3s cluster with role-based access control (RBAC), secrets management, TLS, and best practices for running in a trusted environment.
layout: lesson
type: page
cover: assets/k3s03.jpg
date_updated: 2025-05-24
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
