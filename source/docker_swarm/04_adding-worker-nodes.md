---
title: Adding Worker Nodes to the Swarm
description: This lesson explains how to expand your Docker Swarm by adding more worker nodes to your Raspberry Pi cluster, increasing its capacity and fault tolerance.
layout: lesson
type: page

---

## Expanding Your Docker Swarm Cluster

After initializing Docker Swarm on your Raspberry Pi cluster, adding worker nodes is the next step to increase its processing capacity and reliability. This lesson will guide you through the process of adding new Raspberry Pis as worker nodes to your existing Swarm.

---

### Preparing New Nodes

Before adding new Raspberry Pis to your Swarm, ensure they are:

- Properly set up with the Raspberry Pi OS and have Docker installed (refer to previous lessons for setup instructions).
- Connected to the same network as your Swarm and have internet access.
- Configured with static IP addresses (recommended) to ensure consistent communication within the Swarm.

---

### Using the Swarm Join Token

1. **Retrieve the Join Token**: If you haven't already noted the join token when you initialized the Swarm, you can retrieve it on the manager node by running:

   ```sh
   docker swarm join-token worker
   ```

   This command will output the complete command to join the Swarm as a worker, including the token.

![Docker Swarm Join Token](/learn/docker_swarm/assets/join.png){:class="img-fluid w-100 shadow-lg rounded"}

1. **Join the Swarm**: On each new Raspberry Pi you wish to add as a worker, execute the join command provided by the manager node:

   ```sh
   docker swarm join --token SWMTKN-1-<token_string> <MANAGER_IP>:2377
   ```

   Replace `<token_string>` and `<MANAGER_IP>` with the actual token and IP address of your manager node.

---

### Verifying Worker Nodes

After adding the new worker nodes, you can verify they've successfully joined the Swarm:

- **List Nodes**: On the manager node, run:

  ```sh
  docker node ls
  ```

  This command lists all nodes in the Swarm, including the newly added workers, and shows their status, availability, and role.

---

### Managing Node Roles

Docker Swarm nodes can have either the `manager` or `worker` role:

- **Workers** are responsible for running containers and services but don't participate in Swarm management.
- **Managers** handle orchestration and cluster management tasks but can also run services.

You can dynamically change the role of a node:

- **Promote a Worker to Manager**:

  ```sh
  docker node promote <NODE_ID>
  ```

  This is useful for adding redundancy to your Swarm's management layer.

- **Demote a Manager to Worker**:

  ```sh
  docker node demote <NODE_ID>
  ```

  Use this if you need to reduce the number of manager nodes or to repurpose a node as a worker.

---

### Summary

Adding worker nodes to your Docker Swarm expands the cluster's capacity and fault tolerance, enabling you to run more or larger services. It's a straightforward process that involves using a join token and can be done as your cluster's needs grow. With your Swarm now expanded, you're ready to deploy applications across your Raspberry Pi cluster, leveraging the combined resources of your nodes.

---
