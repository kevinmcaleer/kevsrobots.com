---
title: Adding Nodes
description: Join additional Raspberry Pi nodes to your K3s cluster using the shared token and master node address.
layout: lesson
type: page
cover: assets/k3s05.jpg
date_updated: 2025-05-24
---

![Cover]({{page.cover}}){:class="cover"}

---

With your **K3s control plane (master)** running, it’s time to expand your cluster by **adding worker nodes**.

In this lesson, you’ll SSH into each worker Pi and join it to the K3s cluster using a one-liner join command.

---

## 🧠 How It Works

Worker nodes use:

- The **master node’s IP address**
- A shared **cluster token**
- The `k3s agent` service to register and run workloads

---

## 🔑 Step 1: Copy Your Cluster Token

On your **master node**, run:

```bash
sudo cat /var/lib/rancher/k3s/server/node-token
````

It will look something like:

```text
K108f02c5486e1bdf7a9a7f25a2e7aa3c2c74fa67cced8b31332f5c1b715c962c::server:5bc9c3f0d9249...
```

Copy this token to use on all worker nodes.

---

## 🔧 Step 2: Install K3s Agent on Each Worker Node

On each worker Pi:

1. SSH into the node:

   ```bash
   ssh pi@pi-node1.local
   ```

2. Run the installer with the following environment variables:

   ```bash
   curl -sfL https://get.k3s.io | K3S_URL=https://<MASTER-IP>:6443 K3S_TOKEN=<YOUR_TOKEN> sh -
   ```

   Replace `<MASTER-IP>` and `<YOUR_TOKEN>` with:

   - The IP address of your master node (e.g. `192.168.1.100`)
   - The token you copied earlier

> ✅ Example:
>
> ```bash
> curl -sfL https://get.k3s.io | K3S_URL=https://192.168.1.100:6443 K3S_TOKEN=K108f0... sh -
> ```

---

## 🧪 Step 3: Verify Node Joined the Cluster

Back on the master node:

```bash
kubectl get nodes
```

You should now see your worker(s) listed:

```plaintext
NAME        STATUS   ROLES                  AGE     VERSION
pi-master   Ready    control-plane,master   10m     v1.29.2+k3s1
pi-node1    Ready    <none>                 1m      v1.29.2+k3s1
pi-node2    Ready    <none>                 1m      v1.29.2+k3s1
```

---

## 🛠 Additional Notes

- You can repeat this process for as many worker nodes as you want.
- The install script automatically sets up and starts the `k3s-agent` systemd service.
- Worker nodes will begin running pods as scheduled by the control plane.

---

## 🔍 Troubleshooting

| Issue                     | Fix                                                              |
| ------------------------- | ---------------------------------------------------------------- |
| Node doesn't show up      | Check firewall/ports (`6443` must be open)                       |
| Wrong IP in `K3S_URL`     | Use master’s static IP, not hostname                             |
| Token rejected            | Double-check the token or regenerate with `--token` on reinstall |
| `k3s-agent` service fails | Check logs with `journalctl -u k3s-agent -e`                     |
{:class="table table-striped"}

---

🎉 Your K3s cluster is now multi-node and ready to deploy workloads!

Next up: [Troubleshooting Installation](07_troubleshooting_install)

---
