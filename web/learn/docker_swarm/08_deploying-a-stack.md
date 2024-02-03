---
layout: lesson
title: Deploying a Stack
author: Kevin McAleer
type: page
cover: /learn/docker_swarm/assets/docker_swarm.jpg
date: 2024-02-03
previous: 07_writing-a-docker-compose-file.html
next: 09_managing-and-scaling-services.html
description: Learn how to deploy your Docker Compose-defined applications as a stack
  in Docker Swarm, taking advantage of Swarm's distributed architecture for scalability
  and resilience.
percent: 54
duration: 3
navigation:
- name: Raspberry Pi 5 Cluster with Docker Swarm
- content:
  - section: Introduction
    content:
    - name: Docker Swarm on Raspberry Pi 5
      link: 00_intro.html
  - section: Building the Cluster
    content:
    - name: Planning the Cluster
      link: 01_planning-the-cluster.html
    - name: Cloning Raspberry Pi OS
      link: 02_cloning-raspberry-pi-os.html
    - name: Initializing Docker Swarm
      link: 03_initializing-docker-swarm.html
    - name: Adding Worker Nodes to the Swarm
      link: 04_adding-worker-nodes.html
    - name: Verifying Cluster Setup
      link: 05_verifying-cluster-setup.html
  - section: Deploying Applications
    content:
    - name: Docker Compose and Swarm Stacks
      link: 06_docker-compose-and-swarm-stacks.html
    - name: Writing a Docker Compose File
      link: 07_writing-a-docker-compose-file.html
    - name: Deploying a Stack
      link: 08_deploying-a-stack.html
    - name: Managing and Scaling Services
      link: 09_managing-and-scaling-services.html
    - name: Rebalancing Services in Docker Swarm
      link: 09_rebalancing.html
  - section: Monitoring and Maintenance
    content:
    - name: Monitoring Tools for Docker Swarm
      link: 10_monitoring-tools.html
    - name: Cluster Maintenance
      link: 11_cluster-maintenance.html
    - name: Backup and Recovery Strategies
      link: 12_backup-and-recovery.html
  - section: Conclusion
    content:
    - name: Project Ideas
      link: 13_project-ideas.html
    - name: Further Resources
      link: 14_further-resources.html
---


## Deploying Applications with Docker Swarm Stacks

After defining your multi-container application with a Docker Compose file, the next step is deploying it as a stack in your Docker Swarm cluster. This lesson walks you through the deployment process, highlighting how Docker Swarm interprets your Compose file to distribute services across the cluster.

---

### Understanding Stack Deployment

Deploying a stack in Docker Swarm involves taking a Docker Compose file and using it to set up all the defined services across the nodes in the Swarm. This approach leverages Swarm's orchestration capabilities to manage service scaling, placement, and networking.

---

### Preparing for Deployment

Before deploying your stack, ensure that:

- Your Docker Swarm is initialized and functioning correctly.
- The Docker Compose file is version 3 or above, compatible with Docker Swarm.
- You have access to a manager node in the Swarm to issue deployment commands.

---

### Step-by-Step Deployment Process

1. **Navigate to the Compose File Location**: On the manager node, navigate to the directory containing your Docker Compose file.
1. **Deploy the Stack**:

   - Execute the following command to deploy your stack:

     ```sh
     docker stack deploy -c docker-compose.yml <STACK_NAME>
     ```

   - Replace `<STACK_NAME>` with a name for your stack. This name is used to manage the stack after deployment.
1. **Verify the Deployment**:
   - To check the status of your stack, use:

     ```sh
     docker stack services <STACK_NAME>
     ```

   - This command lists the services within the stack, along with their current state, replicas, and port mappings.

---

### Managing Your Stack

After deployment, Docker Swarm provides several commands to manage your stack:

- **Scaling Services**: Adjust the number of replicas for a service within the stack using the `docker service scale` command.
- **Updating the Stack**: Apply changes to your stack by redeploying it with the same `docker stack deploy` command. Docker Swarm updates services with changed configurations.
- **Removing the Stack**: If you need to remove the stack, use `docker stack rm <STACK_NAME>`. This command stops and removes all services associated with the stack.

---

### Tips for Successful Stack Deployment

- **Resource Allocation**: Consider the resources available on your nodes when deploying services, especially for resource-intensive applications.
- **Service Dependencies**: Use the `depends_on` option in your Compose file wisely to manage service startup order, though be aware it's a soft dependency in Swarm mode.
- **Monitoring and Logging**: Implement logging and monitoring solutions to keep track of your services' health and performance across the cluster.

---

### Summary

Deploying a stack in Docker Swarm transforms your Docker Compose-defined applications into a scalable, resilient system spread across multiple nodes. This lesson covered the essentials of deploying and managing your stack, setting the stage for advanced service orchestration and management strategies in a distributed environment. With your application now running as a stack, you're leveraging the full power of Docker Swarm for your deployment needs.

---
