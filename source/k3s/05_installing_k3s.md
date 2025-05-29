---
title: Installing K3s
description: Install K3s on your Raspberry Pi cluster, configure your master node, and prepare for adding workers.
layout: lesson
type: page
cover: assets/k3s04.jpg
date_updated: 2025-05-24
---

![Cover]({{page.cover}}){:class="cover"}

---

In this lesson, you’ll install **K3s** on your Raspberry Pi master node, set up the cluster, and prepare for adding worker nodes.

K3s makes Kubernetes installation fast and easy — especially on resource-limited devices like Raspberry Pi.

---

## 🚀 Step 1: SSH into the Master Node

From your main computer, SSH into the Pi that will act as the **control plane (master)**:

```bash
ssh pi@pi-master.local
```

> Replace `pi` and `pi-master.local` with your username and hostname or IP if needed.

---

## 🧰 Step 2: Install K3s (Master Node)

Run this one-liner:

```bash
curl -sfL https://get.k3s.io | sh -
```

K3s will:

* Download and install Kubernetes
* Start systemd service: `k3s.service`
* Set up networking with Flannel
* Create a default service account and kubeconfig

---

## 📁 Step 3: Check Cluster Status

After installation:

```bash
sudo k3s kubectl get nodes
```

You should see your master node listed as `Ready`.

To make `kubectl` easier to use, copy the K3s kubeconfig to your user’s home directory:

```bash
mkdir -p ~/.kube
sudo cp /etc/rancher/k3s/k3s.yaml ~/.kube/config
sudo chown $(id -u):$(id -g) ~/.kube/config
```

Now you can run:

```bash
kubectl get nodes
```

---

## 🔑 Step 4: Get the Join Token

You’ll need a token to join worker nodes to the cluster:

```bash
sudo cat /var/lib/rancher/k3s/server/node-token
```

Copy the token — you’ll use it on the worker nodes in the next lesson.

---

## 🧪 Troubleshooting

| Problem                      | Solution                                    |
| ---------------------------- | ------------------------------------------- |
| `k3s.service` fails to start | Check logs: `journalctl -u k3s -e`          |
| `kubectl` not found          | Use `sudo k3s kubectl` or set up kubeconfig |
| Token file missing           | Confirm you're on the master node           |
| Network errors               | Check for cgroup settings and firewalls     |
{:class="table table-striped"}

---

## 📝 Notes

* K3s uses a built-in SQLite database by default (great for small clusters).
* For multi-master HA setups, use an external datastore (e.g. MySQL, Postgres, etcd).
* The default CNI is **Flannel**, which works well for most local setups.

---

🎉 Your Kubernetes control plane is now running!

Next up: [Adding Nodes](06_adding_nodes)

---
