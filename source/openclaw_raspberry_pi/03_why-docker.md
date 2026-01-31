---
title: Why Docker?
description: Understand why Docker is the best way to run OpenClaw on a Raspberry Pi.
layout: lesson
type: page
---

## Why Docker?

You might be wondering: "Why not just install OpenClaw directly on the Pi?" Good question. Let's explore why Docker is the better choice.

### What is Docker?

Docker is containerization software. Think of a container as a lightweight virtual machine that:

- **Packages** your application and all its dependencies
- **Isolates** it from the host system
- **Runs consistently** everywhere it's deployed
- **Uses minimal resources** compared to full VMs

### The Direct Installation Problem

If you install OpenClaw directly on your Pi, you get:

#### Dependency Conflicts
- OpenClaw needs Node.js and specific packages
- Other Pi applications might need different versions
- Managing multiple versions becomes a nightmare

#### System Pollution
- Installation modifies system-wide configuration
- Hard to remove cleanly
- Affects other applications on your Pi

#### Harder Updates
- Need to stop services, update, restart
- Rollback is complicated
- Risk of breaking something

#### Less Isolation
- An OpenClaw bug could affect the entire system
- Permissions can be tricky
- Security concerns with direct file access

### The Docker Advantage

Docker solves all these problems:

#### Isolation
```
Your Pi OS (clean, untouched)
└── Docker
    └── OpenClaw Container (sandboxed)
        ├── Node.js
        ├── OpenClaw code
        ├── All dependencies
        └── Configuration
```

Everything OpenClaw needs lives inside its container. The Pi OS stays clean.

#### Consistent Environment

Container runs identically:
- On your Pi at home
- On another Pi at a friend's house
- On any Linux system
- In the cloud (if you ever wanted to)

The environment is guaranteed.

#### Easy Management

| Task | Direct Install | Docker |
|------|-----------------|--------|
| **Start** | Manual service start | `docker-compose up -d` |
| **Stop** | Manual service stop | `docker-compose down` |
| **Restart** | Kill and restart process | `docker-compose restart` |
| **Update** | Download, extract, restart | `docker-compose pull && up -d` |
| **Rollback** | Complex, manual | Change version, restart |
| **Cleanup** | Leaves files everywhere | `docker-compose down` removes everything |

#### Resource Control

You can limit Docker containers:

```yaml
deploy:
  resources:
    limits:
      memory: 2G
      cpus: '1.5'
```

This prevents OpenClaw from consuming all your Pi's resources even if it goes haywire.

#### Data Persistence

Your OpenClaw workspace persists even if the container is deleted:

```
Delete container (poof!)
├── Configuration is safe
├── Agent memories are safe
└── All your data is safe
```

Restart the container, and everything's still there.

### Docker Compose for Simplicity

Docker has its own complexity. Docker Compose simplifies it.

**Raw Docker command:**
```bash
docker run --name openclaw \
  --restart unless-stopped \
  -p 18789:18789 \
  -v ~/openclaw/workspace:/root/.openclaw/workspace \
  -v ~/openclaw/data:/data \
  -e NODE_ENV=production \
  openclaw/openclaw:latest
```

**Docker Compose:**
```bash
docker-compose up -d
```

One command. The configuration lives in `docker-compose.yml` so you don't have to remember all those flags.

### Performance Considerations

**Does Docker add overhead?**

Minimal:
- Container adds ~20-50MB RAM (negligible)
- CPU overhead is <5% for idle containers
- Disk overhead is minimal

On a Pi with 4GB RAM, you won't notice it.

### Security Benefits

Docker provides security layers:

- **Containerization** - Isolates processes from the host
- **Least privilege** - Container doesn't need root access
- **Network isolation** - Container has its own network namespace
- **Filesystem restrictions** - Container only sees mounted volumes

This makes Docker safer than running OpenClaw with full system access.

### Easy Backups

With Docker:

```bash
# Backup your data
tar -czf backup-$(date +%Y%m%d).tar.gz ~/openclaw/workspace

# Restore anytime
tar -xzf backup-20260131.tar.gz -C ~/
```

Your configuration and data are in one place, easy to backup.

### Running Multiple Instances

Want to run dev and production versions separately?

```
~/openclaw-dev/docker-compose.yml
~/openclaw-prod/docker-compose.yml
```

Each runs independently. No conflicts.

### The Docker Ecosystem

You're not just getting OpenClaw. You're tapping into Docker's huge ecosystem:

- [Docker Hub](https://hub.docker.com) - Thousands of ready-made containers
- [Community support](https://community.docker.com) - Huge community
- [Best practices](https://docs.docker.com) - Extensive documentation
- **Industry standard** - Used by professionals worldwide

Learning Docker on your Pi makes you a better engineer.

### When Not to Use Docker

Docker isn't always the answer:

❌ **Very simple applications** - Just one command, never changing
❌ **GUI applications** - Though possible, it's awkward
❌ **Real-time embedded systems** - Where latency matters most
❌ **Deeply integrated with OS** - Needs deep system access

OpenClaw? Perfect Docker candidate.

### Summary: Docker for OpenClaw

| Aspect | Benefit |
|--------|---------|
| **Isolation** | Clean Pi OS, sandboxed OpenClaw |
| **Updates** | Easy, reversible, atomic |
| **Management** | Simple commands via Compose |
| **Persistence** | Data survives container deletion |
| **Security** | Containerization + least privilege |
| **Backups** | Everything in one place |
| **Learning** | Industry-standard skills |

### Next Steps

Now that you understand why Docker is perfect for this, let's prepare your Raspberry Pi for installation.

---
