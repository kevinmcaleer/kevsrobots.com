# Deployment Guide - KevsRobots Search API

## Overview

This guide covers deploying the KevsRobots Search API to production using Docker on a Raspberry Pi cluster.

---

## Prerequisites

### Hardware
- Raspberry Pi 5 (or 4) - 4x nodes recommended
- Network connectivity between nodes
- External PostgreSQL server

### Software
- Docker 20.10+
- Docker Compose 1.29+
- Git
- Python 3.10+

### Access
- SSH access to all Pi nodes
- PostgreSQL database credentials
- GitHub repository access

---

## Environment Setup

### 1. Clone Repository

```bash
# On build server
cd ~
git clone https://github.com/kevinmcaleer/kevsrobots.com.git
cd kevsrobots.com/search_app
```

### 2. Configure Environment Variables

Create `.env` file in `search_app/` directory:

```bash
# Copy template
cp .env.example .env

# Edit with your credentials
nano .env
```

**Required variables**:

```env
# Application Settings
HOST=0.0.0.0
PORT=8000
ENVIRONMENT=production

# Database Settings
DB_HOST=192.168.2.3
DB_PORT=5433
DB_NAME=searchlogs
DB_USER=your_username
DB_PASSWORD=your_password
DATABASE_URL=postgresql://user:pass@192.168.2.3:5433/searchlogs

# Activity Logging
ENABLE_ACTIVITY_LOGGING=true
```

**Important**:
- Never commit `.env` file to Git
- Use URL-encoded passwords in `DATABASE_URL`
- Ensure PostgreSQL is accessible from Docker containers

---

## PostgreSQL Setup

### 1. Create Database

```bash
# Connect to PostgreSQL
psql -h 192.168.2.3 -p 5433 -U postgres

# Create database
CREATE DATABASE searchlogs;

# Create user (if needed)
CREATE USER search_user WITH PASSWORD 'your_password';

# Grant permissions
GRANT ALL PRIVILEGES ON DATABASE searchlogs TO search_user;

# Exit
\q
```

### 2. Verify Connection

```bash
# Test connection
psql -h 192.168.2.3 -p 5433 -U search_user -d searchlogs

# Should connect successfully
```

### 3. Initialize Tables

The application will automatically create the `search_logs` table on first run, but you can create it manually:

```sql
CREATE TABLE search_logs (
    id SERIAL PRIMARY KEY,
    timestamp TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    client_ip VARCHAR(45) NOT NULL,
    query TEXT NOT NULL,
    results_count INTEGER,
    execution_time FLOAT,
    page INTEGER DEFAULT 1,
    page_size INTEGER DEFAULT 10,
    user_agent TEXT,
    referer TEXT
);

CREATE INDEX idx_search_logs_timestamp ON search_logs(timestamp);
CREATE INDEX idx_search_logs_client_ip ON search_logs(client_ip);
CREATE INDEX idx_search_logs_query ON search_logs(query);
```

---

## Docker Build

### 1. Review Dockerfile

The `search_app/dockerfile` should contain:

```dockerfile
FROM python:3.11-slim

WORKDIR /app

# Install dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application
COPY . .

# Expose port
EXPOSE 8000

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD curl -f http://127.0.0.1:8000/health || exit 1

# Run application
CMD ["uvicorn", "app:app", "--host", "0.0.0.0", "--port", "8000"]
```

### 2. Build Docker Image

```bash
# On build server
cd ~/kevsrobots.com/search_app

# Build for ARM64 (Raspberry Pi)
docker build --platform linux/arm64 -t search-api:latest .

# Tag for registry
docker tag search-api:latest 192.168.2.1:5000/search-api:latest
```

### 3. Push to Private Registry

```bash
# Push to Pi registry
docker push 192.168.2.1:5000/search-api:latest
```

---

## Deployment to Raspberry Pi Nodes

### 1. Create Docker Compose File

Create `docker-compose.yml`:

```yaml
version: '3.8'

services:
  search-api:
    image: 192.168.2.1:5000/search-api:latest
    container_name: search-api
    restart: unless-stopped
    ports:
      - "8000:8000"
    env_file:
      - .env
    volumes:
      - ./search.db:/app/search.db
    healthcheck:
      test: ["CMD", "curl", "-f", "http://127.0.0.1:8000/health"]
      interval: 30s
      timeout: 10s
      retries: 3
      start_period: 5s
    networks:
      - search-network

networks:
  search-network:
    driver: bridge
```

### 2. Deploy to Each Node

```bash
# SSH to each Pi node (dev01, dev02, dev03, dev04)
ssh pi@dev01

# Navigate to deployment directory
cd ~/search_app

# Copy .env file (if not present)
# Create docker-compose.yml

# Pull latest image
docker-compose pull

# Stop existing container
docker-compose down

# Start new container
docker-compose up -d

# Check status
docker-compose ps
docker-compose logs -f
```

### 3. Verify Deployment

```bash
# Check health
curl http://localhost:8000/health

# Expected output:
# {"status":"healthy","service":"KevsRobots Search API","version":"2.0.0","timestamp":"..."}

# Test search
curl "http://localhost:8000/search/?query=robot"

# Check logs
docker logs search-api
```

---

## Load Balancer Configuration

### Nginx Configuration

Create `/etc/nginx/sites-available/search-api`:

```nginx
upstream search_backend {
    least_conn;
    server 192.168.2.11:8000 max_fails=3 fail_timeout=30s;
    server 192.168.2.12:8000 max_fails=3 fail_timeout=30s;
    server 192.168.2.13:8000 max_fails=3 fail_timeout=30s;
    server 192.168.2.14:8000 max_fails=3 fail_timeout=30s;
}

server {
    listen 80;
    server_name search.kevsrobots.local;

    location / {
        proxy_pass http://search_backend;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;

        # Timeouts
        proxy_connect_timeout 60s;
        proxy_send_timeout 60s;
        proxy_read_timeout 60s;

        # Buffers
        proxy_buffering on;
        proxy_buffer_size 8k;
        proxy_buffers 16 8k;
    }

    # Health check endpoint
    location /health {
        access_log off;
        proxy_pass http://search_backend/health;
    }
}
```

Enable and reload:

```bash
sudo ln -s /etc/nginx/sites-available/search-api /etc/nginx/sites-enabled/
sudo nginx -t
sudo systemctl reload nginx
```

---

## Cloudflare Setup

### 1. Add DNS Record

In Cloudflare dashboard:
- Type: `A`
- Name: `@` or subdomain
- IPv4: Your public IP
- Proxy status: Proxied (orange cloud)
- TTL: Auto

### 2. Configure Page Rules

**Cache Everything**:
- URL: `www.kevsrobots.com/search/*`
- Cache Level: Standard
- Edge Cache TTL: 2 hours

**Origin Server Rules**:
- Health checks enabled
- URL: `https://www.kevsrobots.com/health`
- Interval: 60 seconds

### 3. SSL/TLS Settings

- SSL/TLS encryption mode: Full
- Always Use HTTPS: On
- Minimum TLS Version: 1.2

---

## Monitoring & Logging

### 1. View Application Logs

```bash
# Real-time logs
docker logs -f search-api

# Last 100 lines
docker logs --tail 100 search-api

# Logs since 1 hour ago
docker logs --since 1h search-api
```

### 2. PostgreSQL Logs

```bash
# Check recent searches
psql -h 192.168.2.3 -p 5433 -U search_user -d searchlogs

SELECT timestamp, client_ip, query, results_count, execution_time
FROM search_logs
ORDER BY timestamp DESC
LIMIT 20;
```

### 3. Health Monitoring

```bash
# Check all nodes
for host in dev01 dev02 dev03 dev04; do
    echo "Checking $host..."
    curl -s http://$host:8000/health | jq
done
```

---

## Backup Strategy

### 1. PostgreSQL Backup

Create backup script `/usr/local/bin/backup-searchlogs.sh`:

```bash
#!/bin/bash

BACKUP_DIR="/backup/searchlogs"
DATE=$(date +%Y%m%d_%H%M%S)
FILENAME="searchlogs_${DATE}.sql"

# Create backup directory
mkdir -p $BACKUP_DIR

# Dump database
pg_dump -h 192.168.2.3 -p 5433 -U search_user -d searchlogs > "${BACKUP_DIR}/${FILENAME}"

# Compress
gzip "${BACKUP_DIR}/${FILENAME}"

# Keep only last 30 days
find $BACKUP_DIR -name "*.sql.gz" -mtime +30 -delete

echo "Backup completed: ${FILENAME}.gz"
```

Make executable and add to cron:

```bash
chmod +x /usr/local/bin/backup-searchlogs.sh

# Add to crontab (daily at 2 AM)
crontab -e
0 2 * * * /usr/local/bin/backup-searchlogs.sh
```

### 2. SQLite Backup

```bash
#!/bin/bash

BACKUP_DIR="/backup/search-db"
DATE=$(date +%Y%m%d_%H%M%S)

mkdir -p $BACKUP_DIR

# Copy SQLite database
docker exec search-api sqlite3 /app/search.db ".backup /app/search_backup.db"
docker cp search-api:/app/search_backup.db "${BACKUP_DIR}/search_${DATE}.db"

# Compress
gzip "${BACKUP_DIR}/search_${DATE}.db"

# Keep only last 7 days
find $BACKUP_DIR -name "*.db.gz" -mtime +7 -delete
```

---

## Rollback Procedure

### Quick Rollback

```bash
# On each Pi node

# Pull previous version
docker pull 192.168.2.1:5000/search-api:previous

# Update docker-compose.yml to use :previous tag
# Or manually run:
docker run -d \
    --name search-api \
    --env-file .env \
    -p 8000:8000 \
    192.168.2.1:5000/search-api:previous

# Verify health
curl http://localhost:8000/health
```

---

## Troubleshooting

### Container Won't Start

```bash
# Check logs
docker logs search-api

# Common issues:
# 1. Database connection failed - check .env file
# 2. Port already in use - check with `netstat -tlnp | grep 8000`
# 3. Missing .env file - create from template
```

### Database Connection Errors

```bash
# Test PostgreSQL connection
psql -h 192.168.2.3 -p 5433 -U search_user -d searchlogs

# Check if DB_PASSWORD is URL-encoded in .env
# Special characters need encoding: @ → %40, # → %23, etc.
```

### Slow Search Performance

```bash
# Check SQLite database size
ls -lh search.db

# Optimize SQLite
docker exec search-api sqlite3 /app/search.db "VACUUM;"
docker exec search-api sqlite3 /app/search.db "ANALYZE;"

# Rebuild FTS5 index
docker exec search-api sqlite3 /app/search.db "INSERT INTO documents_fts(documents_fts) VALUES('optimize');"
```

### Memory Issues

```bash
# Check container memory usage
docker stats search-api

# Limit memory if needed (add to docker-compose.yml)
services:
  search-api:
    mem_limit: 512m
    memswap_limit: 512m
```

---

## Security Checklist

- [ ] `.env` file not committed to Git
- [ ] PostgreSQL on private network only
- [ ] Strong database passwords used
- [ ] Cloudflare DDoS protection enabled
- [ ] CORS origins properly configured
- [ ] Regular backups scheduled
- [ ] Container health checks enabled
- [ ] Nginx timeouts configured
- [ ] SSL/TLS enabled via Cloudflare
- [ ] Docker registry secured

---

## Performance Optimization

### 1. Database Indexes

Ensure indexes exist:

```sql
-- Check existing indexes
SELECT indexname, indexdef
FROM pg_indexes
WHERE tablename = 'search_logs';

-- Add missing indexes
CREATE INDEX IF NOT EXISTS idx_search_logs_timestamp ON search_logs(timestamp);
CREATE INDEX IF NOT EXISTS idx_search_logs_client_ip ON search_logs(client_ip);
CREATE INDEX IF NOT EXISTS idx_search_logs_query ON search_logs(query);
```

### 2. Connection Pooling (Future)

Consider implementing connection pooling with SQLAlchemy:

```python
from sqlalchemy import create_engine
from sqlalchemy.pool import QueuePool

engine = create_engine(
    DATABASE_URL,
    poolclass=QueuePool,
    pool_size=10,
    max_overflow=20
)
```

### 3. Caching (Future)

Consider adding Redis for caching popular searches:

```yaml
services:
  redis:
    image: redis:alpine
    ports:
      - "6379:6379"
```

---

## Maintenance Schedule

### Daily
- Monitor error logs
- Check health endpoints

### Weekly
- Review search analytics
- Check disk space
- Review slow queries

### Monthly
- Backup verification
- Update Docker images
- Security updates
- Performance review

### Quarterly
- Database optimization
- Archive old logs
- Capacity planning
- Security audit

---

## Update Procedure

### Rolling Update

```bash
# 1. Build new image
cd ~/kevsrobots.com/search_app
git pull
docker build --platform linux/arm64 -t search-api:latest .
docker tag search-api:latest 192.168.2.1:5000/search-api:latest
docker push 192.168.2.1:5000/search-api:latest

# 2. Update one node at a time
ssh pi@dev01
docker-compose pull
docker-compose up -d
# Wait for health check
curl http://localhost:8000/health

# 3. Repeat for remaining nodes
# This ensures zero downtime
```

---

## Support & Contacts

**Repository**: https://github.com/kevinmcaleer/kevsrobots.com
**Issues**: https://github.com/kevinmcaleer/kevsrobots.com/issues
**Email**: kevinmcaleer@gmail.com
