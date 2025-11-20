# Database Schema Documentation

## Overview

The KevsRobots Search API uses two separate databases for different purposes:

1. **SQLite** - Full-text search index (read-optimized)
2. **PostgreSQL** - Search query logging and analytics (write-optimized)

This separation of concerns allows for optimal performance in both search execution and analytics.

---

## PostgreSQL Database: searchlogs

**Host**: `192.168.2.3`
**Port**: `5433`
**Database**: `searchlogs`

### Table: search_logs

Stores all search queries with metadata for analytics and monitoring.

#### Schema

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
```

#### Indexes

```sql
CREATE INDEX idx_search_logs_timestamp ON search_logs(timestamp);
CREATE INDEX idx_search_logs_client_ip ON search_logs(client_ip);
CREATE INDEX idx_search_logs_query ON search_logs(query);
```

#### Column Descriptions

| Column | Type | Nullable | Default | Description |
|--------|------|----------|---------|-------------|
| `id` | SERIAL | NO | AUTO | Primary key, auto-incrementing |
| `timestamp` | TIMESTAMP | NO | CURRENT_TIMESTAMP | When the search occurred |
| `client_ip` | VARCHAR(45) | NO | - | IPv4 or IPv6 address of client |
| `query` | TEXT | NO | - | The search query string |
| `results_count` | INTEGER | YES | NULL | Number of results returned |
| `execution_time` | FLOAT | YES | NULL | Query execution time in seconds |
| `page` | INTEGER | YES | 1 | Pagination: current page number |
| `page_size` | INTEGER | YES | 10 | Pagination: results per page |
| `user_agent` | TEXT | YES | NULL | Browser/client user agent string |
| `referer` | TEXT | YES | NULL | HTTP referer (source page) |

#### Data Types Explained

- **SERIAL**: Auto-incrementing integer (PostgreSQL-specific)
- **TIMESTAMP**: Date and time (YYYY-MM-DD HH:MM:SS)
- **VARCHAR(45)**: Variable character string, max 45 chars (supports IPv6)
- **TEXT**: Variable unlimited text
- **INTEGER**: Whole number
- **FLOAT**: Decimal number (double precision)

#### Example Data

```sql
id  | timestamp           | client_ip      | query        | results_count | execution_time | page | page_size | user_agent                    | referer
----|---------------------|----------------|--------------|---------------|----------------|------|-----------|-------------------------------|------------------
1   | 2025-11-20 14:30:15 | 192.168.1.100  | robot        | 42            | 0.123          | 1    | 10        | Mozilla/5.0...                | https://google.com
2   | 2025-11-20 14:31:22 | 192.168.1.101  | micropython  | 18            | 0.089          | 1    | 10        | Mozilla/5.0...                | https://www.kevsrobots.com
3   | 2025-11-20 14:32:45 | 192.168.1.100  | robot        | 42            | 0.115          | 2    | 10        | Mozilla/5.0...                | https://www.kevsrobots.com/search
```

---

## SQLite Database: search.db

**Location**: `search_app/search.db`
**Type**: SQLite3 with FTS5 extension

### Table: documents_fts

Full-text search virtual table for document indexing.

#### Schema

```sql
CREATE VIRTUAL TABLE documents_fts
USING fts5(
    title,
    content,
    url,
    cover_image,
    page_title,
    description,
    date,
    author
);
```

#### Column Descriptions

| Column | Type | Description |
|--------|------|-------------|
| `title` | TEXT | Document title |
| `content` | TEXT | Full document content (searchable) |
| `url` | TEXT | Document URL (unique identifier) |
| `cover_image` | TEXT | URL to cover image |
| `page_title` | TEXT | Page-specific title |
| `description` | TEXT | Document description/summary |
| `date` | TEXT | Publication date |
| `author` | TEXT | Document author |

#### FTS5 Features

- **Full-text indexing**: Automatically indexes all text columns
- **Ranking**: Results ordered by relevance (BM25 algorithm)
- **Fast searches**: Optimized for text search queries
- **Match highlighting**: Can highlight matched terms
- **Phrase searches**: Support for exact phrase matching

#### Example Query

```sql
-- Search for documents containing "robot"
SELECT DISTINCT url, cover_image, page_title, description, date, author
FROM documents_fts
WHERE documents_fts MATCH 'robot'
GROUP BY url
ORDER BY rank
LIMIT 10 OFFSET 0;
```

---

## Database Access Patterns

### Write Operations

#### PostgreSQL (search_logs)
- **Frequency**: Every search request
- **Operation**: INSERT
- **Volume**: ~1000-5000 writes/day (estimated)
- **Performance**: ~5-10ms per insert

#### SQLite (documents_fts)
- **Frequency**: During content updates only
- **Operation**: INSERT/UPDATE
- **Volume**: ~10-100 writes/day (estimated)
- **Performance**: ~1-2ms per insert

### Read Operations

#### PostgreSQL (search_logs)
- **Frequency**: Analytics requests only
- **Operation**: SELECT with aggregations
- **Volume**: ~10-50 reads/day (estimated)
- **Performance**: ~50-200ms per query

#### SQLite (documents_fts)
- **Frequency**: Every search request
- **Operation**: FTS5 MATCH query
- **Volume**: ~1000-5000 reads/day (estimated)
- **Performance**: ~50-150ms per query

---

## Index Strategy

### PostgreSQL Indexes

1. **idx_search_logs_timestamp**
   - Column: `timestamp`
   - Purpose: Fast filtering by date ranges
   - Used by: Popular searches, statistics queries

2. **idx_search_logs_client_ip**
   - Column: `client_ip`
   - Purpose: Track searches per IP
   - Used by: IP-based analytics, rate limiting

3. **idx_search_logs_query**
   - Column: `query`
   - Purpose: Fast aggregation by query string
   - Used by: Popular searches query

### SQLite Indexes

FTS5 automatically creates its own inverted index structure. No manual indexes needed.

---

## Data Retention Policy

### Current Policy
- **search_logs**: Unlimited retention
- **documents_fts**: Permanent storage

### Recommended Future Policy

```sql
-- Delete logs older than 1 year
DELETE FROM search_logs
WHERE timestamp < NOW() - INTERVAL '1 year';

-- Or archive old data
CREATE TABLE search_logs_archive AS
SELECT * FROM search_logs
WHERE timestamp < NOW() - INTERVAL '1 year';

DELETE FROM search_logs
WHERE timestamp < NOW() - INTERVAL '1 year';
```

---

## Backup Strategy

### PostgreSQL Backup

```bash
# Daily backup
pg_dump -h 192.168.2.3 -p 5433 -U username -d searchlogs > backup_$(date +%Y%m%d).sql

# Restore from backup
psql -h 192.168.2.3 -p 5433 -U username -d searchlogs < backup_20251120.sql
```

### SQLite Backup

```bash
# Simple file copy (when app is not running)
cp search.db search.db.backup

# Using SQLite backup command
sqlite3 search.db ".backup search.db.backup"
```

---

## Database Initialization

### PostgreSQL

The `search_logger.py` module automatically creates the table on first run:

```python
from search.search_logger import get_search_logger

# This will create the table if it doesn't exist
logger = get_search_logger()
```

### SQLite

Run the database module to initialize:

```bash
cd search_app
python -c "from search.database import initialize_database; initialize_database()"
```

Or from the module:

```python
from search.database import initialize_database
initialize_database()
```

---

## Migration Scripts

### Adding a new column to search_logs

```sql
-- Add column for tracking search category
ALTER TABLE search_logs
ADD COLUMN category VARCHAR(50);

-- Add index on new column
CREATE INDEX idx_search_logs_category ON search_logs(category);
```

### Performance Tuning

```sql
-- Analyze table statistics
ANALYZE search_logs;

-- View table size
SELECT pg_size_pretty(pg_total_relation_size('search_logs'));

-- View index usage
SELECT
    schemaname,
    tablename,
    indexname,
    idx_scan,
    idx_tup_read,
    idx_tup_fetch
FROM pg_stat_user_indexes
WHERE tablename = 'search_logs';
```

---

## Security Considerations

### PostgreSQL
1. **Network isolation**: Database on private network (192.168.2.x)
2. **Authentication**: Username/password from environment variables
3. **Access control**: Limited to application servers only
4. **Encryption**: Connection encryption recommended (SSL/TLS)

### SQLite
1. **File permissions**: Restrict read/write to application user
2. **Backup encryption**: Encrypt backup files
3. **No network exposure**: Local file only

---

## Monitoring Queries

### Check recent search activity

```sql
SELECT
    DATE_TRUNC('hour', timestamp) as hour,
    COUNT(*) as search_count
FROM search_logs
WHERE timestamp >= NOW() - INTERVAL '24 hours'
GROUP BY hour
ORDER BY hour DESC;
```

### Find slow queries

```sql
SELECT
    query,
    AVG(execution_time) as avg_time,
    MAX(execution_time) as max_time,
    COUNT(*) as query_count
FROM search_logs
WHERE execution_time > 0.5
GROUP BY query
ORDER BY avg_time DESC
LIMIT 20;
```

### Most active IPs

```sql
SELECT
    client_ip,
    COUNT(*) as search_count,
    COUNT(DISTINCT query) as unique_queries
FROM search_logs
WHERE timestamp >= NOW() - INTERVAL '7 days'
GROUP BY client_ip
ORDER BY search_count DESC
LIMIT 20;
```

---

## Database Maintenance

### PostgreSQL Maintenance Tasks

```sql
-- Vacuum to reclaim space
VACUUM search_logs;

-- Analyze to update statistics
ANALYZE search_logs;

-- Reindex if needed
REINDEX TABLE search_logs;
```

### SQLite Maintenance Tasks

```sql
-- Optimize database
VACUUM;

-- Rebuild FTS5 index
INSERT INTO documents_fts(documents_fts) VALUES('optimize');

-- Check integrity
PRAGMA integrity_check;
```
