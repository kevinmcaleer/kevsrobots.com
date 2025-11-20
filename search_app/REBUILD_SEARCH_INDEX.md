# Rebuild Search Index Guide

## Problem

Search returns no results because the SQLite FTS (Full-Text Search) database is empty or outdated.

## Solution

The search index must be rebuilt by parsing all HTML files from the Jekyll-generated `_site` directory.

---

## Quick Rebuild

### Option 1: Using the Indexer Script (Recommended)

```bash
# From the repository root
cd ~/kevsrobots.com

# Make sure Jekyll site is built
cd stacks
docker-compose up -d
# Wait for build to complete (check logs)
docker-compose logs -f jekyll-serve

# Go back to search_app
cd ~/kevsrobots.com/search_app

# Run the indexer
python3 index.py
```

### Option 2: Automated Rebuild Script

I've created a convenience script that does everything:

```bash
cd ~/kevsrobots.com/search_app
./rebuild_search_index.sh
```

---

## How It Works

### 1. Jekyll Builds Static Site

```
source/ → Jekyll → web/_site/
  ├── courses/          ├── index.html
  ├── blog posts/       ├── learn/
  └── pages/            └── blog/
```

### 2. Indexer Parses HTML

The `index.py` script:
- Walks through `web/_site/` directory
- Parses every `.html` file
- Extracts: title, content, metadata, images
- Inserts into SQLite FTS database

### 3. Search API Queries Database

```
Client Search → FastAPI → SQLite FTS5 → Results
```

---

## Manual Rebuild Process

### Step 1: Ensure Jekyll Site is Built

```bash
cd ~/kevsrobots.com/stacks
docker-compose ps

# If not running
docker-compose up -d

# Check build completion
docker-compose logs -f jekyll-serve
# Press Ctrl+C when you see "Server running..."
```

Verify the site exists:
```bash
ls -la ~/kevsrobots.com/web/_site/
# Should show: index.html, learn/, blog/, assets/, etc.
```

### Step 2: Clear Old Index (Optional)

```bash
cd ~/kevsrobots.com/search_app

# Backup existing database
cp search.db search.db.backup.$(date +%Y%m%d)

# Remove old database
rm search.db
```

### Step 3: Initialize New Database

```bash
cd ~/kevsrobots.com/search_app
python3 -c "from search.database import initialize_database; initialize_database()"
```

### Step 4: Run Indexer

```bash
cd ~/kevsrobots.com/search_app
python3 index.py
```

This will:
- Walk through all HTML files
- Parse and extract content
- Insert into SQLite FTS table

**Progress indicators**: You won't see much output unless there are errors.

### Step 5: Verify Index

```bash
# Check database size (should be several MB)
ls -lh search.db

# Count indexed documents
sqlite3 search.db "SELECT COUNT(DISTINCT url) FROM documents_fts;"

# Test a search
sqlite3 search.db "SELECT url, page_title FROM documents_fts WHERE documents_fts MATCH 'robot' LIMIT 5;"
```

### Step 6: Test API

```bash
# Start the API (if not running)
uvicorn app:app --reload

# Test search
curl "http://localhost:8000/search/?query=robot"
```

---

## Automated Rebuild Script

I've created `rebuild_search_index.sh` that automates all these steps:

```bash
#!/bin/bash

echo "KevsRobots Search Index Rebuild"
echo "================================"
echo ""

# Step 1: Check if Jekyll site exists
if [ ! -d "../web/_site" ]; then
    echo "ERROR: Jekyll site not found at ../web/_site"
    echo "Build the Jekyll site first:"
    echo "  cd ~/kevsrobots.com/stacks"
    echo "  docker-compose up -d"
    exit 1
fi

# Step 2: Count HTML files
HTML_COUNT=$(find ../web/_site -name "*.html" | wc -l)
echo "Found $HTML_COUNT HTML files to index"
echo ""

# Step 3: Backup existing database
if [ -f "search.db" ]; then
    BACKUP_NAME="search.db.backup.$(date +%Y%m%d_%H%M%S)"
    echo "Backing up existing database to: $BACKUP_NAME"
    cp search.db "$BACKUP_NAME"
    echo ""
fi

# Step 4: Remove old database
if [ -f "search.db" ]; then
    echo "Removing old database..."
    rm search.db
    echo ""
fi

# Step 5: Initialize new database
echo "Initializing new database..."
python3 -c "from search.database import initialize_database; initialize_database()"
echo ""

# Step 6: Run indexer
echo "Indexing HTML files..."
echo "This may take a minute..."
python3 index.py

# Step 7: Verify
echo ""
echo "Verifying index..."
DOC_COUNT=$(sqlite3 search.db "SELECT COUNT(DISTINCT url) FROM documents_fts;")
echo "Indexed $DOC_COUNT unique documents"

# Step 8: Test search
echo ""
echo "Testing search for 'robot'..."
RESULT_COUNT=$(sqlite3 search.db "SELECT COUNT(*) FROM documents_fts WHERE documents_fts MATCH 'robot';")
echo "Found $RESULT_COUNT results for 'robot'"

echo ""
echo "================================"
echo "Rebuild complete!"
echo ""
echo "Test the API:"
echo "  curl \"http://localhost:8000/search/?query=robot\""
```

---

## Troubleshooting

### No HTML files found

**Problem**: `find ../web/_site -name "*.html"` returns nothing

**Solution**: Build the Jekyll site first
```bash
cd ~/kevsrobots.com/stacks
docker-compose up -d
```

### Permission denied errors

**Problem**: Can't write to search.db

**Solution**: Check file permissions
```bash
ls -la search.db
chmod 644 search.db
```

### Indexer fails with import error

**Problem**: `ModuleNotFoundError: No module named 'search'`

**Solution**: Run from correct directory
```bash
cd ~/kevsrobots.com/search_app
python3 index.py
```

### Search still returns no results

**Problem**: Database exists but is empty

**Check 1**: Verify database has data
```bash
sqlite3 search.db "SELECT COUNT(*) FROM documents_fts;"
# Should return > 0
```

**Check 2**: Check index.py path
```python
# In index.py, check this line:
for root, dirs, files in os.walk('../web/_site'):
```

Make sure path is correct relative to where you run the script.

**Check 3**: Manual test
```bash
sqlite3 search.db
sqlite> .tables
sqlite> SELECT COUNT(*) FROM documents_fts;
sqlite> SELECT * FROM documents_fts LIMIT 1;
```

### Database locked error

**Problem**: `database is locked`

**Solution**: Stop the API before rebuilding
```bash
# Find the process
ps aux | grep uvicorn

# Kill it
kill <PID>

# Or if running in Docker
docker-compose down
```

---

## Scheduled Rebuilds

### Option 1: Cron Job

Rebuild nightly at 2 AM:

```bash
crontab -e

# Add this line:
0 2 * * * cd /home/user/kevsrobots.com/search_app && ./rebuild_search_index.sh >> /tmp/search_rebuild.log 2>&1
```

### Option 2: After Jekyll Build

Add to your Jekyll build script:

```bash
#!/bin/bash
# build_and_deploy.sh

# Build Jekyll site
cd ~/kevsrobots.com/stacks
docker-compose up -d

# Wait for build
sleep 30

# Rebuild search index
cd ~/kevsrobots.com/search_app
./rebuild_search_index.sh

# Deploy (your deployment steps here)
```

---

## Production Deployment

When deploying to production, you need the search.db file:

### Copy Database to Container

**Docker Compose Method**:

```yaml
# docker-compose.yml
services:
  search-api:
    volumes:
      - ./search.db:/app/search.db:ro  # :ro = read-only
```

**Manual Copy Method**:

```bash
# Copy into running container
docker cp search.db search-api:/app/search.db

# Restart container
docker restart search-api
```

### Build Into Image (Not Recommended)

You can build the database into the Docker image, but this means rebuilding the entire container when search index changes:

```dockerfile
# In Dockerfile
COPY search.db /app/search.db
```

**Better**: Mount as volume (as shown above).

---

## Monitoring

### Check Index Freshness

```bash
# Last modified time
ls -lh search.db

# File size
du -h search.db

# Document count
sqlite3 search.db "SELECT COUNT(DISTINCT url) FROM documents_fts;"
```

### Alert if Index is Old

```bash
#!/bin/bash
# check_search_index.sh

DB_AGE=$(stat -f %m search.db)
NOW=$(date +%s)
AGE_HOURS=$(( ($NOW - $DB_AGE) / 3600 ))

if [ $AGE_HOURS -gt 48 ]; then
    echo "WARNING: Search index is $AGE_HOURS hours old"
    echo "Consider rebuilding: ./rebuild_search_index.sh"
fi
```

---

## Summary

1. **Jekyll builds** HTML files to `web/_site/`
2. **index.py parses** HTML and inserts into SQLite FTS
3. **search.db** contains searchable index
4. **FastAPI queries** the database for searches

**To rebuild**:
```bash
cd ~/kevsrobots.com/search_app
./rebuild_search_index.sh
```

**To verify**:
```bash
curl "http://localhost:8000/search/?query=robot"
```

Done!
