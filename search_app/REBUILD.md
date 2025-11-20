# Quick Rebuild Guide

## Issue Fixed

Updated Dockerfile to include PostgreSQL dependencies (`libpq-dev`, `gcc`) required for `psycopg2-binary`.

## Rebuild Steps

### Option 1: Using Build Script (Recommended)

```bash
cd ~/kevsrobots.com/search_app

# Make script executable (if needed)
chmod +x build_app.sh

# Run build script (builds and pushes to registry)
./build_app.sh
```

### Option 2: Manual Build

```bash
cd ~/kevsrobots.com/search_app

# Build for ARM64 (Raspberry Pi)
docker build --platform linux/arm64 -t 192.168.2.1:5000/search:latest .

# Push to registry
docker push 192.168.2.1:5000/search:latest
```

## Deploy to Nodes

After rebuilding, deploy to each Pi node:

```bash
# SSH to each node (dev01, dev02, dev03, dev04)
ssh pi@dev01

# Pull latest image
docker pull 192.168.2.1:5000/search:latest

# Restart container
docker-compose down
docker-compose up -d

# Verify health
curl http://localhost:8000/health

# Check logs
docker logs -f search-api
```

## Verify PostgreSQL Logging

Test that searches are being logged:

```bash
# Make a test search
curl "http://localhost:8000/search/?query=test"

# Check PostgreSQL logs
psql -h 192.168.2.3 -p 5433 -U your_user -d searchlogs

# View recent logs
SELECT * FROM search_logs ORDER BY timestamp DESC LIMIT 5;
```

## Troubleshooting

### Still getting psycopg2 error?

1. Check that requirements.txt contains `psycopg2-binary`
2. Rebuild without cache: `docker build --no-cache -t 192.168.2.1:5000/search:latest .`
3. Verify Python packages in container: `docker exec search-api pip list | grep psycopg2`

### Health check failing?

```bash
# Check container logs
docker logs search-api

# Check if app is running
docker exec search-api curl http://127.0.0.1:8000/health

# Verify .env file exists
docker exec search-api ls -la /usr/src/app/.env
```

### Database connection error?

1. Verify `.env` file has correct credentials
2. Test PostgreSQL connection: `psql -h 192.168.2.3 -p 5433 -U your_user -d searchlogs`
3. Check if database is accessible from Docker network
4. Review app logs: `docker logs search-api 2>&1 | grep -i postgres`

## Quick Test Commands

```bash
# Health check
curl http://localhost:8000/health

# Search test
curl "http://localhost:8000/search/?query=robot"

# Recent searches (should show your test search)
curl "http://localhost:8000/analytics/recent?limit=5"

# Container status
docker ps | grep search-api

# Container health
docker inspect search-api | grep -A 5 Health
```
