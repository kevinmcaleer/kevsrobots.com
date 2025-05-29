---
title: Ingress and Services
description: Learn how to expose Kubernetes applications to your network using Services and Ingress controllers in K3s.
layout: lesson
type: page
cover: assets/k3s04.jpg
date_updated: 2025-05-24
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
