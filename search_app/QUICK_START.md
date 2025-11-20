# Quick Start Guide - Search API Setup

## Complete Setup in 3 Steps

### 1. Migrate Historical Logs to PostgreSQL

```bash
cd ~/kevsrobots.com/search_app

# Test timestamp preservation (optional but recommended)
python3 test_timestamp_preservation.py

# Dry run first (see what would be migrated)
python3 migrate_logs_to_postgres.py --dry-run

# Actually migrate
python3 migrate_logs_to_postgres.py
```

**What this does**: Reads `search-logs.log` and imports all historical searches into PostgreSQL.

**IMPORTANT**: The migration **preserves historical timestamps**! If your log has entries from 2023, they'll be inserted with 2023 timestamps, not today's date. This means your analytics will show when searches actually occurred.

**Expected output**:
```
Migrating logs from: search-logs.log
--------------------------------------------------------------------------------
Inserted 100 records...
Inserted 200 records...
...
================================================================================
MIGRATION SUMMARY
================================================================================
Total lines read:     1,234
Successfully parsed:  1,234
Skipped (invalid):    0
Inserted to DB:       1,234
Errors:               0
================================================================================
Success rate: 100.0%
```

### 2. Rebuild Search Index

```bash
cd ~/kevsrobots.com/search_app

# Make sure Jekyll site is built first
cd ../stacks
docker-compose ps  # Check if running
# If not running: docker-compose up -d

# Rebuild the search index
cd ~/kevsrobots.com/search_app
./rebuild_search_index.sh
```

**What this does**: Indexes all HTML content from Jekyll site into SQLite FTS database.

**Expected output**:
```
========================================
KevsRobots Search Index Rebuild
========================================

📄 Found 1,523 HTML files to index
💾 Backing up existing database
🔧 Initializing new database
📚 Indexing HTML files...
✅ Indexing complete

🔍 Verifying index...
   📊 Statistics:
      • Unique documents: 856
      • Total entries: 1,523
      • Database size: 12M

🧪 Testing search functionality...
   ✅ 'robot': found 142 results
   ✅ 'micropython': found 68 results
   ✅ 'raspberry': found 95 results
```

### 3. Test Everything

```bash
# Start the API (if not already running)
cd ~/kevsrobots.com/search_app
uvicorn app:app --reload --host 0.0.0.0 --port 8000

# In another terminal, test:

# 1. Health check
curl http://localhost:8000/health

# 2. Search test
curl "http://localhost:8000/search/?query=robot"

# 3. Check analytics (historical data)
curl "http://localhost:8000/analytics/stats?days=365"

# 4. Check recent searches (should include your test)
curl "http://localhost:8000/analytics/recent?limit=5"
```

---

## Verify PostgreSQL Data

```bash
# Check historical data was migrated
psql -h 192.168.2.3 -p 5433 -U your_user -d searchlogs

# Run queries
SELECT COUNT(*) FROM search_logs;
SELECT MIN(timestamp), MAX(timestamp) FROM search_logs;
SELECT query, COUNT(*) FROM search_logs GROUP BY query ORDER BY COUNT(*) DESC LIMIT 10;
```

---

## Verify Search Index

```bash
cd ~/kevsrobots.com/search_app

# Check database exists and size
ls -lh search.db

# Count documents
sqlite3 search.db "SELECT COUNT(DISTINCT url) FROM documents_fts;"

# Test search
sqlite3 search.db "SELECT url, page_title FROM documents_fts WHERE documents_fts MATCH 'robot' LIMIT 5;"
```

---

## Deploy to Production

### Step 1: Build New Docker Image

```bash
cd ~/kevsrobots.com/search_app
./build_app.sh
```

This includes all the fixes:
- PostgreSQL dependencies
- URL-decoded credentials
- Real IP extraction from proxy headers

### Step 2: Copy Search Database

```bash
# Copy search.db to each Pi node
for host in dev01 dev02 dev03 dev04; do
    echo "Copying to $host..."
    scp search.db pi@$host:~/search_app/
done
```

### Step 3: Deploy to Nodes

```bash
# On each node
ssh pi@dev01

cd ~/search_app
docker-compose pull
docker-compose down
docker-compose up -d

# Verify
curl http://localhost:8000/health
curl "http://localhost:8000/search/?query=robot"
```

---

## Troubleshooting

### Migration Issues

**No .env file**
```bash
cd ~/kevsrobots.com/search_app
cp .env.example .env
nano .env  # Add your database credentials
```

**Connection failed**
```bash
# Test PostgreSQL connection
psql -h 192.168.2.3 -p 5433 -U your_user -d searchlogs

# If fails, check .env credentials
python3 decode_credentials.py
```

### Search Index Issues

**No HTML files found**
```bash
# Build Jekyll site
cd ~/kevsrobots.com/stacks
docker-compose up -d
docker-compose logs -f jekyll-serve
```

**Search returns no results**
```bash
# Verify database has data
sqlite3 search.db "SELECT COUNT(*) FROM documents_fts;"

# Should return > 0
# If 0, run: ./rebuild_search_index.sh
```

**Permission denied**
```bash
chmod 644 search.db
chown $USER:$USER search.db
```

---

## Maintenance Schedule

### Daily
- Monitor search logs in PostgreSQL
- Check API health endpoints

### Weekly
- Review popular searches
- Check for errors in logs

### Monthly
- Rebuild search index (after content updates)
```bash
cd ~/kevsrobots.com/search_app
./rebuild_search_index.sh
```

### As Needed
- Migrate new log files (if switching servers)
- Update Docker images (after code changes)

---

## Common Commands Reference

### Migration
```bash
# Dry run migration
python3 migrate_logs_to_postgres.py --dry-run

# Actual migration
python3 migrate_logs_to_postgres.py

# Migrate specific file
python3 migrate_logs_to_postgres.py --log-file /path/to/other.log
```

### Search Index
```bash
# Rebuild index
./rebuild_search_index.sh

# Manual rebuild
python3 -c "from search.database import initialize_database; initialize_database()"
python3 index.py
```

### Testing
```bash
# Test search
curl "http://localhost:8000/search/?query=robot"

# Test analytics
curl "http://localhost:8000/analytics/popular?days=30"
curl "http://localhost:8000/analytics/stats?days=7"
```

### Database
```bash
# PostgreSQL
psql -h 192.168.2.3 -p 5433 -U your_user -d searchlogs -c "SELECT COUNT(*) FROM search_logs;"

# SQLite
sqlite3 search.db "SELECT COUNT(*) FROM documents_fts;"
```

---

## What's Next?

1. ✅ Historical logs migrated to PostgreSQL
2. ✅ Search index rebuilt
3. ✅ API working locally
4. ⏳ Deploy to production
5. ⏳ Set up monitoring
6. ⏳ Schedule regular index rebuilds

**See also**:
- `REBUILD.md` - Full rebuild and deployment guide
- `documentation/API.md` - API endpoint reference
- `documentation/DEPLOYMENT.md` - Production deployment
- `documentation/PROXY_HEADERS.md` - Nginx configuration
- `REBUILD_SEARCH_INDEX.md` - Detailed search index guide

---

## Support

**Issues**: https://github.com/kevinmcaleer/kevsrobots.com/issues
**Email**: kevinmcaleer@gmail.com
