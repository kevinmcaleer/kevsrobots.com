# Troubleshooting: No Search Results in Production

## Problem

Search returns no results for "python" (or other queries) in production, but works locally.

## Quick Diagnosis

Run these commands on your production server:

### Step 1: Copy debug script to production
```bash
# From your local machine
scp debug_production.sh pi@dev01:~/search_app/
ssh pi@dev01
cd ~/search_app
chmod +x debug_production.sh
./debug_production.sh
```

### Step 2: Check container database
```bash
# Check if database exists in container
docker exec search-api ls -lh /usr/src/app/search.db

# Verify database inside container
docker exec search-api python3 /usr/src/app/verify_database.py

# Check what the app sees
docker exec -it search-api bash
cd /usr/src/app
ls -lh search.db
python3 verify_database.py
exit
```

---

## Common Causes & Fixes

### 1. Database Not Uploaded to Container

**Problem**: You uploaded search.db to the host, but the container doesn't see it.

**Check**:
```bash
# On host
ls -lh ~/search_app/search.db

# In container
docker exec search-api ls -lh /usr/src/app/search.db
```

**Fix**: Copy database into container or use volume mount

**Option A - Copy into container**:
```bash
docker cp search.db search-api:/usr/src/app/search.db
docker restart search-api
```

**Option B - Use volume mount** (recommended):

Edit `docker-compose.yml`:
```yaml
services:
  search-api:
    volumes:
      - ./search.db:/usr/src/app/search.db:ro  # Read-only
```

Then:
```bash
docker-compose down
docker-compose up -d
```

---

### 2. Database is Empty or Corrupted

**Check**:
```bash
docker exec search-api python3 << 'EOF'
import sqlite3
conn = sqlite3.connect('/usr/src/app/search.db')
cursor = conn.cursor()
cursor.execute("SELECT COUNT(*) FROM documents_fts")
print(f"Total entries: {cursor.fetchone()[0]}")
cursor.execute("SELECT COUNT(*) FROM documents_fts WHERE documents_fts MATCH 'python'")
print(f"Python results: {cursor.fetchone()[0]}")
conn.close()
EOF
```

**Expected output**:
```
Total entries: 1113
Python results: 1041
```

**If 0 entries**: Database is empty, need to rebuild/reupload

**Fix**:
```bash
# On local machine where database works
cd ~/kevsrobots.com/search_app
python3 verify_database.py  # Verify local DB has data

# Copy to production
scp search.db pi@dev01:~/search_app/

# On production
ssh pi@dev01
cd ~/search_app
docker cp search.db search-api:/usr/src/app/search.db
docker restart search-api

# Verify
docker exec search-api python3 verify_database.py
```

---

### 3. Wrong Database Path

**Problem**: App is looking at wrong location or creating new empty database.

**Check**:
```bash
# Find all search.db files in container
docker exec search-api find / -name "search.db" 2>/dev/null
```

**Expected**: Only `/usr/src/app/search.db`

**If multiple found**: App might be creating new empty database

**Fix**: Check database path in code
```python
# In search/database.py
SEARCH_DB = 'search.db'  # Relative to /usr/src/app

# Or use absolute path:
SEARCH_DB = '/usr/src/app/search.db'
```

---

### 4. Permissions Issue

**Check**:
```bash
docker exec search-api ls -l /usr/src/app/search.db
```

**Should be readable** (at least `r--` for group/others)

**Fix**:
```bash
docker exec search-api chmod 644 /usr/src/app/search.db
docker restart search-api
```

---

### 5. Index Built from Wrong _site Directory

**Problem**: Index was built from an old or incomplete Jekyll _site

**Check local build**:
```bash
# Verify local database has data
python3 verify_database.py

# Check Jekyll site has files
ls -la ../web/_site/ | wc -l
# Should show many files (hundreds)

# Check for HTML files
find ../web/_site -name "*.html" | wc -l
# Should match number indexed (532 in your case)
```

**Fix**: Rebuild index from correct Jekyll site
```bash
# Make sure Jekyll site is fresh
cd ~/kevsrobots.com/stacks
docker-compose restart jekyll-serve

# Rebuild search index
cd ~/kevsrobots.com/search_app
./rebuild_search_index.sh

# Verify
python3 verify_database.py

# Upload to production
scp search.db pi@dev01:~/search_app/
```

---

## Complete Fix Process

### 1. Verify Local Database
```bash
cd ~/kevsrobots.com/search_app
python3 verify_database.py
```

Should show:
- ✅ 1113 total entries
- ✅ 1041 results for 'python'

### 2. Copy to Production
```bash
# Copy database
scp search.db pi@dev01:~/search_app/

# Copy debug scripts
scp verify_database.py debug_production.sh pi@dev01:~/search_app/
```

### 3. Install on Production
```bash
ssh pi@dev01
cd ~/search_app

# Verify file was copied
ls -lh search.db

# Verify it has data
python3 verify_database.py

# Stop container
docker-compose down

# Copy into container location (if needed)
# Option 1: Volume mount (recommended)
# Make sure docker-compose.yml has:
#   volumes:
#     - ./search.db:/usr/src/app/search.db:ro

# Start container
docker-compose up -d

# Wait for health check
sleep 10

# Verify container can see database
docker exec search-api python3 verify_database.py

# Test API
curl "http://localhost:8000/search/?query=python" | jq '.total_count'
# Should return: 1041
```

### 4. If Still No Results

Check app logs:
```bash
docker logs search-api | tail -50
```

Test database connection from inside container:
```bash
docker exec -it search-api bash
cd /usr/src/app
python3
>>> import sqlite3
>>> conn = sqlite3.connect('search.db')
>>> cursor = conn.cursor()
>>> cursor.execute("SELECT COUNT(*) FROM documents_fts WHERE documents_fts MATCH 'python'")
>>> print(cursor.fetchone()[0])
>>> exit()
exit
```

---

## Quick Test Commands

```bash
# Test local database
python3 verify_database.py

# Test production database on host
ssh pi@dev01 'cd ~/search_app && python3 verify_database.py'

# Test production database in container
ssh pi@dev01 'docker exec search-api python3 verify_database.py'

# Test API
curl "http://localhost:8000/search/?query=python" | jq '.total_count'
```

---

## Prevention

### 1. Use Docker Volume Mount

In `docker-compose.yml`:
```yaml
services:
  search-api:
    volumes:
      - ./search.db:/usr/src/app/search.db:ro
```

This ensures the container always uses the host's search.db file.

### 2. Verify Before Deploy

Always run `python3 verify_database.py` before uploading to production.

### 3. Automate Deployment

Create `deploy_database.sh`:
```bash
#!/bin/bash
set -e

echo "Verifying local database..."
python3 verify_database.py || exit 1

echo "Copying to production..."
scp search.db pi@dev01:~/search_app/

echo "Restarting container..."
ssh pi@dev01 'cd ~/search_app && docker-compose restart search-api'

echo "Waiting for health check..."
sleep 10

echo "Verifying production..."
ssh pi@dev01 'docker exec search-api python3 verify_database.py'

echo "Testing API..."
curl -s "http://dev01:8000/search/?query=python" | jq '.total_count'

echo "✅ Deployment complete!"
```

---

## Still Having Issues?

1. Check container logs: `docker logs search-api`
2. Verify Jekyll site is complete: `ls -la ../web/_site/`
3. Rebuild index from scratch: `./rebuild_search_index.sh`
4. Check file sizes match: `ls -lh search.db` (should be ~19MB)
5. Try absolute path in `search/database.py`: `SEARCH_DB = '/usr/src/app/search.db'`
