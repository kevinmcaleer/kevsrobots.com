---
title: Writing a Docker Compose File
description: >-
    Write a stack file for Swarm with a web service and a cache, using the deploy key for replicas, rolling updates, limits and placement.
layout: lesson
type: page
cover: /learn/docker_swarm/assets/banners/07_writing-a-docker-compose-file.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to write a stack file with the `deploy:` settings that tell Swarm how many copies to run, how to update them and where to put them.

---

## The shape of a stack file

A stack file has the same top-level sections as any Compose file:

- `services:` the containers you want, each with an `image:`, `ports:` and so on
- `volumes:` named storage
- `networks:` extra networks (Swarm creates a default one for the stack)
- `configs:` and `secrets:` small files Swarm hands to containers

What makes it a *swarm* file is the `deploy:` key inside each service. Docker Compose on a single machine ignores most of it. `docker stack deploy` reads all of it.

---

## Our example: a web service and a cache

Save this as `stack.yml` on a manager. Both images are official and are published for `arm64`:

```yaml
services:
  web:
    image: nginx:alpine
    ports:
      - "8180:80"
    healthcheck:
      test: ["CMD", "wget", "-q", "-O", "/dev/null", "http://127.0.0.1/"]
      interval: 10s
      timeout: 3s
      retries: 3
    deploy:
      replicas: 3
      update_config:
        parallelism: 1
        delay: 10s
        order: start-first
        failure_action: rollback
      restart_policy:
        condition: on-failure
      resources:
        limits:
          memory: 64M

  cache:
    image: redis:7-alpine
    volumes:
      - cache-data:/data
    deploy:
      replicas: 1
      placement:
        constraints:
          - node.role == manager

volumes:
  cache-data:
```

Let's walk through it.

### `image` and `ports`

`web` runs the small `nginx:alpine` image. `"8180:80"` publishes container port 80 on port 8180 of **every** node, thanks to the routing mesh from lesson 5.

### `healthcheck`

Swarm only sends traffic to, and counts as ready, a task whose health check passes. Here every ten seconds we fetch the home page from inside the container. We use `127.0.0.1` rather than `localhost` to avoid IPv6 surprises.

### `deploy: replicas`

Three copies of `web`. If one dies, Swarm starts another to get back to three.

### `deploy: update_config`

How a rolling update behaves. `parallelism: 1` changes one task at a time, `delay: 10s` waits between them, `order: start-first` starts the new task before stopping the old one so nothing goes dark, and `failure_action: rollback` automatically goes back if the new version fails.

### `deploy: restart_policy` and `resources`

`on-failure` restarts crashed tasks. `limits: memory: 64M` stops one greedy container from starving the whole Pi.

### `deploy: placement`

Constraints pin a service to certain nodes. Our cache uses a **named volume**, and a named volume lives on one node's disk only. If Swarm moved the cache to another node, it would start with an empty volume. Pinning it to the manager keeps the data where it is. This is the most common surprise for newcomers, and we revisit it in the backup lesson.

---

## Check your file before deploying

Swarm will not interpret every key as you expect, so preview it:

```bash
docker stack config -c stack.yml
```

This prints the file as Docker understands it, and complains about syntax errors. Indentation in YAML matters: use spaces, never tabs.

---

## Secrets, not passwords in files

Notice we put no password anywhere. For a real database you would store it as a Swarm **secret**, created with `docker secret create`, which containers read from a file under `/run/secrets/`. It is far safer than an `environment:` line committed to Git. We don't use one in this stack, but the Try it Yourself below has you explore it.

---

## Try it Yourself

1. Change `replicas: 3` to `replicas: 4` and `"8180:80"` to `"8181:80"`, then run `docker stack config -c stack.yml` and spot the change.
2. Add `cpus: "0.50"` under `resources: limits:` for `web` and check the file still validates.
3. Challenge: read about `docker secret create` and sketch how you would give a `postgres` service its password without writing it in `stack.yml`.

---

## Common Issues

- **Problem**: `yaml: line 12: found character that cannot start any token` or similar.
- **Solution**: Replace tab characters with spaces and keep each level indented by two spaces.
- **Why**: YAML uses indentation for structure and forbids tabs.

- **Problem**: The cache loses its data after a node is rebooted or the task moves.
- **Solution**: Pin it with a `placement` constraint, or use shared storage such as NFS.
- **Why**: A named volume is stored on a single node, so a task placed elsewhere starts with a fresh empty volume.

- **Problem**: The service never becomes healthy.
- **Solution**: Test the health command by hand with `docker exec <container> wget -q -O /dev/null http://127.0.0.1/`.
- **Why**: A wrong command or port makes the check fail forever, and Swarm keeps restarting the task.

---

## Check your understanding

{% quiz %}
? What does `order: start-first` do during an update?
- [ ] Starts all tasks at once
- [x] Starts the new task before stopping the old one
- [ ] Updates the manager first
> `start-first` means there is always a task running during the update, so there is no gap in service.

? Why did we pin `cache` to the manager?
- [ ] Managers have faster disks
- [x] A named volume lives on one node, so the task must stay where the data is
- [ ] Redis only runs on managers
> Local volumes are not shared between nodes, so a moved task would find an empty volume.

? Which command previews how Docker reads your stack file?
- [x] `docker stack config -c stack.yml`
- [ ] `docker stack lint`
- [ ] `docker compose rm`
> `docker stack config` prints the processed file and reports errors before you deploy.
{% endquiz %}
