---
title: Storage and Volumes
description: Learn how storage works in Kubernetes, how to use persistent volumes in K3s, and set up storage that survives pod restarts.
layout: lesson
type: page
cover: assets/k3s03.jpg
date_updated: 2025-05-24
---

![Cover]({{page.cover}}){:class="cover"}

---

In Kubernetes, storage isn’t tied to a container like in Docker — it’s managed independently as **persistent volumes (PVs)** and **persistent volume claims (PVCs)**.

This lesson will show you how **storage works in K3s**, how to use the **built-in local-path provisioner**, and how to prepare for **external storage options** (like NFS or USB SSDs).

---

## 💾 How Storage Works in K3s

K3s includes a **default dynamic storage provisioner** called `local-path-provisioner`.

- It uses local disk storage on the node running the pod
- PVCs are automatically provisioned
- Easy to use, but not shared across nodes

> ⚠️ If a pod moves to a different node, its data may not follow.

---

## 📁 Step 1: Test with a Simple PVC

Create a file `pvc-demo.yaml`:

```yaml
apiVersion: v1
kind: PersistentVolumeClaim
metadata:
  name: demo-pvc
spec:
  accessModes:
    - ReadWriteOnce
  resources:
    requests:
      storage: 1Gi
````

Apply it:

```bash
kubectl apply -f pvc-demo.yaml
```

---

## 📦 Step 2: Mount PVC in a Pod

Create a pod with a volume mount:

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: pvc-tester
spec:
  containers:
    - name: busybox
      image: busybox
      command: [ "sleep", "3600" ]
      volumeMounts:
        - mountPath: "/data"
          name: myvolume
  volumes:
    - name: myvolume
      persistentVolumeClaim:
        claimName: demo-pvc
```

Apply it:

```bash
kubectl apply -f pvc-tester.yaml
```

Then exec into the pod and test:

```bash
kubectl exec -it pvc-tester -- sh
# Inside pod:
echo "Hello Pi Cluster" > /data/test.txt
cat /data/test.txt
```

---

## 🧠 Where Is the Data Stored?

On the node hosting the pod, the data is typically under:

```bash
/opt/local-path-provisioner
```

Each PVC is stored in its own directory.

---

## 📤 External Storage Options (Optional)

For more advanced setups:

| Option     | Benefit                        | Tool                                  |
| ---------- | ------------------------------ | ------------------------------------- |
| USB SSD    | Fast, local storage            | Mount manually or with Longhorn       |
| NFS Server | Shared across all nodes        | Use `nfs-subdir-external-provisioner` |
| Ceph       | Highly available block storage | More complex to set up                |
{:class="table table-striped"}

> 🛠 Consider using **Longhorn** if you want persistent block storage with replication and snapshots.

---

## 🧪 Cleanup

To remove test volumes and pods:

```bash
kubectl delete pod pvc-tester
kubectl delete pvc demo-pvc
```

---

## ✅ Summary

You now know how to:

- Create persistent volume claims in K3s
- Use local-path provisioner for basic storage needs
- Mount volumes into pods
- Plan for external and scalable storage options

---

Next up: [Ingress and Services](11_ingress_and_services)

---
