---
title: Docker Swarm
description: >-
    Learn how to build a Docker Swarm cluster on Raspberry Pi 5, and deploy and manage scalable microservices applications.
layout: showcase
date: 2024-02-03
author: Kevin McAleer
difficulty: intermediate
excerpt: >-
    Learn how to build a Docker Swarm cluster on Raspberry Pi 5, and deploy and manage scalable microservices applications.
cover: /assets/img/blog/docker_swarm/docker_swarm.jpg
hero: /assets/img/blog/docker_swarm/hero.png
tags:
  - Cluster
  - Docker
mode: light
groups:
  - raspberrypi
videos:
  - tDENgLiJSh0
code:
  - https://www.github.com/kevinmcaleer/ClusteredPi
---

If you have more than one Raspberry Pi, you can combine the power of all of them to create a `Cluster`. A cluster is a group of computers that work together to solve a problem. In this case, we will use a cluster of Raspberry Pi computers to create a `Docker Swarm` cluster.

A Docker Swarm cluster is a group of `Docker` hosts that run in a cluster mode. 

Clusters can provide high availability and failover, and can be used to spread the workload across many `nodes` in the cluster where each Raspberry Pi is a separate node. This means that you can add more Raspberry Pi computers to the cluster to increase the capacity of the cluster.

In this project, we will create a Docker Swarm cluster on Raspberry Pi 5, and deploy and manage scalable `microservices` applications.

> ## What is a Microservice?
>
> A `microservice` is a small, independent, and loosely coupled service that is designed to perform a single task. Microservices are designed to be small and lightweight, and can be deployed and scaled independently. Microservices are often used to build large, complex applications that are composed of many small, independent services.

---

## What you will need

- 3 or more Raspberry Pi 4 or Raspberry Pi 5 computers with the 64 Bit Raspberry Pi OS installed
- A network switch
- A network cable for each Raspberry Pi
- A microSD card for each Raspberry Pi 4, or an NVMe SSD for each Raspberry Pi 5, with an NVMe Hat or Base
- A power supply for each Raspberry Pi (the 27W power supply is recommended for the Pi 5)
- A keyboard, mouse, and monitor for the initial setup of each Raspberry Pi

---

## Step 1: Install Docker on each Raspberry Pi

The first step is to install `Docker` on each Raspberry Pi. Docker is a platform for developing, shipping, and running applications. Docker allows you to package your application and all of its dependencies into a single container that can be run on any Docker host.

For a quick refresher on how to install Docker, check out the free [Docker](/learn/docker/) course.

---

## Step 2: Set up the network

The next step is to set up the network for the Docker Swarm cluster. You will need a network switch and a network cable for each Raspberry Pi. Connect each Raspberry Pi to the network switch using a network cable.

![A cluster of 4 Raspberry Pi 5s with cables connected to a switch](/assets/img/blog/docker_swarm/network.jpg){:class="img-fluid w-100 rounded-3 shadow-lg"}

Once you have connected each Raspberry Pi to the network switch, you will need to configure the network settings on each Raspberry Pi. You will need to give each Raspberry Pi a `static` IP address, and configure the network settings to use the same subnet mask and default gateway.

> ## What is a Static IP Address?
>
> A `static` IP address is an IP address that is manually assigned to a device, and does not change. A static IP address is used to ensure that a device always has the same IP address, and is often used for devices that need to be accessed over the network.
>
> A static IP address is used to ensure that a device always has the same IP address, and is often used for devices that need to be accessed over the network.
>
> ## How to setup a static IP address on a Raspberry Pi
>
> To set up a static IP address on a Raspberry Pi, you will need to edit the `dhcpcd.conf` file. The `dhcpcd.conf` file is used to configure the network settings on a Raspberry Pi, and can be used to set up a static IP address.
>
> To set up a static IP address on a Raspberry Pi, you will need to create a new file called `/etc/systemd/network/10-eth0.network` and add the following lines to the file:
>
> ```bash
> [Match]
> Name=eth0
>
> [Network]
> Address=192.168.2.1 # or whatever address you want
> Gateway=192.168.1.254 # or whatever your gateway address is
> DNS=192.168.1.254 # or whatever your DNS server address is
> ```
>
> Once you have added the lines to the file, you will need to restart the network service with the following command:
>
> ```bash
> sudo systemctl enable systemd-networkd
> sudo systemctl restart systemd-networkd
> ```

---

## Step 3: Create the Docker Swarm cluster

The next step is to create the Docker Swarm cluster.

A Docker Swarm cluster is a group of Docker hosts that run in a cluster mode. To create the Docker Swarm cluster, you will need to run a few commands on the Docker host that you want to use as the `Swarm` manager.

To initialize the Docker Swarm cluster, you will need to run the following command on the Docker host that you want to use as the Swarm manager:

```bash
docker swarm init
```

Notice the join instructions - you can cut and paste these into the other Raspberry Pi's to join them to the cluster.

---

> ## What is a Docker Swarm Manager?
>
> The Docker Swarm `Manager` is the node in the Docker Swarm cluster that is responsible for managing the cluster and scheduling workloads. The Docker Swarm manager is the node that you will use to create and manage the Docker Swarm cluster.
>
> The Docker Swarm manager is responsible for the following tasks:
>
> - Managing the cluster
> - Scheduling workloads
> - Managing the state of the cluster
> - Managing the state of the services
> - Managing the state of the nodes
> - Managing the state of the tasks
>
> ---
>
> ## What is a Docker Swarm Worker?
>
> The Docker Swarm `Worker` is the node in the Docker Swarm cluster that is responsible for running the workloads. The Docker Swarm worker is the node that will run the containers that make up the services in the Docker Swarm cluster.
>
> The Docker Swarm worker is responsible for the following tasks:
>
> - Running the workloads
> - Running the containers
> - Running the services
> - Running the tasks

---

## Step 4: Deploy and manage scalable microservices applications

The final step is to deploy and manage scalable microservices applications on the Docker Swarm cluster. A microservices application is a large, complex application that is composed of many small, independent services. Each service in the microservices application is designed to perform a single task, and can be deployed and scaled independently.

To deploy and manage scalable microservices applications on the Docker Swarm cluster, you will need to create a `Docker Compose` file that describes the services in the microservices application, and then use the `docker stack` command to deploy the services to the Docker Swarm cluster.

I've created a couple of docker-compose files that I use to host kevsrobots.com, and you can use these as a starting point for your own projectsl clone <https://www.github.com/kevinmcaleer/ClusteredPi/> and look in the Stack folder for the files.

---

## Conclusion

In this project, we created a Docker Swarm cluster on Raspberry Pi 5, and deployed and managed scalable microservices applications. We learned how to install Docker on each Raspberry Pi, set up the network for the Docker Swarm cluster, create the Docker Swarm cluster, and deploy and manage scalable microservices applications on the Docker Swarm cluster.