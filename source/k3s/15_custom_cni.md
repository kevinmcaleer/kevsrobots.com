---
title: Advanced Networking & Custom CNI
description: Explore Kubernetes Container Network Interfaces (CNI) in K3s and learn how to replace the default Flannel with more advanced CNIs like Calico or Cilium.
layout: lesson
type: page
cover: assets/k3s02.jpg
date_updated: 2025-05-24
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
