---
title: Docker Compose and Swarm Stacks
description: Learn the difference between Docker Compose and Swarm Stacks, and how to utilize them for deploying applications on your Docker Swarm cluster.
layout: lesson
type: page

---

## Introduction to Deployment Tools

Deploying applications on a Docker Swarm cluster requires understanding the tools available for managing service configurations and orchestrations. This lesson introduces Docker Compose and Swarm Stacks, explaining their purposes, differences, and how to use them effectively in a Swarm environment.

---

### Understanding Docker Compose

Docker Compose is a tool for defining and running multi-container Docker applications. With Compose, you use a YAML file to configure your application's services, networks, and volumes. Then, with a single command, you create and start all the services defined in your configuration.

- **Primary Use**: Ideal for development, testing, and staging environments, as well as single-host production deployments.
- **Key Features**:
  - Simple, declarative YAML configuration for services.
  - Manages the entire application lifecycle with simple commands.
  - Supports variables and extension fields for flexible configurations.

---

### Introduction to Docker Swarm Stacks

Swarm Stacks are similar to Docker Compose but are specifically designed for managing applications across a Docker Swarm cluster. Stacks allow you to deploy, update, and scale services across multiple Docker hosts within the Swarm.

- **Primary Use**: Best suited for production environments and multi-host deployments in a Docker Swarm.
- **Key Features**:
  - Utilizes Docker Compose files (version 3 and above) for service definitions.
  - Seamlessly integrates with Docker Swarm for native clustering support.
  - Enables service replication across nodes, load balancing, and secure service-to-service communication.

---

### Differences Between Docker Compose and Swarm Stacks

| Feature | Docker Compose | Swarm Stacks |
|---------|----------------|--------------|
| Scope   | Single host    | Multi-host (Swarm) |
| Use Case| Development, testing, and small-scale production | Large-scale production deployments in Swarm |
| Command | `docker-compose up` | `docker stack deploy` |
| File Format | Docker Compose YAML (supports version 2 and 3) | Docker Compose version 3 YAML, with Swarm-specific options |
{:class="table table-striped"}

---

### Deploying a Stack to Docker Swarm

1. **Create a Docker Compose File**: Define your application stack in a `docker-compose.yml` file, using version 3 syntax to ensure compatibility with Docker Swarm.
1. **Deploy the Stack**:

   - Run the following command on a manager node to deploy the stack:

     ```sh
     docker stack deploy -c docker-compose.yml <STACK_NAME>
     ```

   - Replace `<STACK_NAME>` with a name for your stack.
1. **Verify Deployment**:
   - Check the status of your stack with:

     ```sh
     docker stack services <STACK_NAME>
     ```

   - This command lists the services in your stack, along with their replicas and status.

---

## How to update a docker service to the latest version

You may often need to update docker services with the latest version of a container. One of quirks of docker is that it does not automatically update the containers to the latest version, when you do `docker service update <SERVICENAME>`. You need to follow the process below, in this example a service called `kevsrobots` is updated to the latest version.

1. Build (or Rebuild) and tag the image

   ```bash
   docker build -t 192.168.2.1:5000/kevsrobots:latest .
   ```

1. Push the image
   After rebuilding your image, tag it (even if you're reusing the latest tag) and push it to the registry. 

    ```bash
   
    docker push 192.168.2.1:5000/kevsrobots:latest
    ```

1. Update the Service
   To update the service and force all nodes to pull the latest image, use the --image flag with the docker service update command, specifying the full image name, including the tag:

    ```bash
    docker service update --image 192.168.2.1:5000/kevsrobots:latest kevsrobots
    ```

---

## Additional Tips

- **Force Pull**: If you want to explicitly ensure that Docker Swarm pulls the image on each node, you can use the --with-registry-auth flag to pass registry authentication credentials to the Swarm nodes. This is particularly useful if your image is stored in a private registry that requires authentication.

  ```bash
  docker service update --image 192.168.2.1:5000/kevsrobots:latest --with-registry-auth kevsrobots
  ```

- **No Downtime**: Ensure you have configured the service with an appropriate update policy to achieve zero downtime deployments. You can set parameters like --update-parallelism and --update-delay to control the rollout of the update across the service's tasks.

  ```bash
  docker service update --image 192.168.2.1:5000/kevsrobots:latest --update-parallelism 1 --update-delay 10s kevsrobots
  ```

- **Rollback Strategy**: It's also a good practice to configure a rollback strategy in case the new version of the image introduces issues.

  ```bash
  docker service update --image 192.168.2.1:5000/kevsrobots:latest --rollback-parallelism 1 --rollback-delay 10s kevsrobots
  ```

By following these steps, you can ensure that all nodes in your Docker Swarm pull and use the latest version of your image, maintaining the consistency of your deployment.

---

### Summary

Docker Compose and Swarm Stacks provide powerful and flexible tools for deploying and managing containerized applications. Understanding when and how to use each tool is crucial for effectively managing your Docker Swarm cluster. By leveraging Docker Stacks, you can take full advantage of Docker Swarm's capabilities for deploying scalable and resilient applications across multiple nodes in your cluster.

---
