# Data Flow Documentation

## Overview

This document describes how data flows through the KevsRobots Search API system, from initial request to final response.

---

## Search Request Flow

### 1. Client Search Request

```
User enters search query → Browser/Client sends HTTP GET request
```

**Example Request**:
```http
GET /search/?query=micropython&page=1&page_size=10 HTTP/1.1
Host: www.kevsrobots.com
User-Agent: Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7)
Referer: https://www.kevsrobots.com/
```

**Data Captured**:
- Query string: `micropython`
- Page number: `1`
- Page size: `10`
- Client IP: `192.168.1.100`
- User Agent: `Mozilla/5.0...`
- Referer: `https://www.kevsrobots.com/`

---

### 2. Load Balancer Routing

```
Cloudflare CDN → Nginx Load Balancer → FastAPI Instance
```

**Cloudflare Processing**:
- SSL/TLS termination
- DDoS protection
- Caching (if configured)
- Geographic routing

**Nginx Processing**:
- Health check validation
- Round-robin distribution
- Timeout management
- Proxy headers forwarding

---

### 3. FastAPI Request Handling

**File**: `app.py`
**Endpoint**: `@app.get("/search/")`

```python
async def search_documents(request: Request, query: str, page: int = 1, page_size: int = 10):
    # Step 1: Capture request metadata
    start_time = time.time()
    client_ip = request.client.host
    user_agent = request.headers.get("user-agent")
    referer = request.headers.get("referer")
```

**Data Processing**:
1. Extract query parameters
2. Capture timing
3. Extract client metadata
4. Validate inputs

---

### 4. SQLite Search Execution

**File**: `search/database.py`
**Function**: `query_documents()`

```python
# Calculate pagination offset
offset = (page - 1) * page_size  # (1 - 1) * 10 = 0

# Execute FTS5 query
results = query_documents(query="micropython", offset=0, limit=10)
```

**SQLite Query**:
```sql
SELECT DISTINCT url, cover_image, page_title, description, date, author
FROM documents_fts
WHERE documents_fts MATCH 'micropython'
GROUP BY url
ORDER BY rank
LIMIT 10 OFFSET 0;
```

**Data Retrieved**:
```json
[
    {
        "url": "/learn/micropython/",
        "cover_image": "/assets/micropython.jpg",
        "page_title": "MicroPython Introduction",
        "description": "Learn MicroPython basics",
        "date": "2024-01-15",
        "author": "Kevin McAleer"
    },
    // ... more results
]
```

---

### 5. Result Count Query

**Function**: `total_results()`

```sql
SELECT COUNT(*) FROM documents_fts
WHERE documents_fts MATCH 'micropython';
```

**Result**: `18` (total matching documents)

---

### 6. PostgreSQL Logging

**File**: `search/search_logger.py`
**Function**: `log_search()`

```python
log_id = search_logger.log_search(
    client_ip="192.168.1.100",
    query="micropython",
    results_count=18,
    execution_time=0.123,
    page=1,
    page_size=10,
    user_agent="Mozilla/5.0...",
    referer="https://www.kevsrobots.com/"
)
```

**PostgreSQL INSERT**:
```sql
INSERT INTO search_logs
    (client_ip, query, results_count, execution_time, page, page_size, user_agent, referer)
VALUES
    ('192.168.1.100', 'micropython', 18, 0.123, 1, 10, 'Mozilla/5.0...', 'https://www.kevsrobots.com/')
RETURNING id;
```

**Result**: `log_id = 12345`

---

### 7. Response Assembly

```python
execution_time = time.time() - start_time  # 0.123 seconds

return {
    "results": results,              # Array of 10 documents
    "total_count": 18,               # Total matches
    "total_pages": 2,                # 18 / 10 = 2 pages
    "page": 1,                       # Current page
    "page_size": 10,                 # Results per page
    "execution_time": 0.123          # Rounded to 3 decimals
}
```

---

### 8. JSON Response

**HTTP Response**:
```http
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 1234

{
    "results": [
        {
            "url": "/learn/micropython/",
            "cover_image": "/assets/micropython.jpg",
            "page_title": "MicroPython Introduction",
            "description": "Learn MicroPython basics",
            "date": "2024-01-15",
            "author": "Kevin McAleer"
        },
        // ... 9 more results
    ],
    "total_count": 18,
    "total_pages": 2,
    "page": 1,
    "page_size": 10,
    "execution_time": 0.123
}
```

---

## Complete Search Flow Diagram

```
┌──────────┐
│  Client  │
└────┬─────┘
     │ GET /search/?query=micropython&page=1
     │
     ▼
┌────────────┐
│ Cloudflare │
└────┬───────┘
     │
     ▼
┌────────────┐
│   Nginx    │
└────┬───────┘
     │
     ▼
┌────────────────────────────────────┐
│         FastAPI App                │
│  ┌──────────────────────────────┐  │
│  │ 1. Extract request metadata  │  │
│  │    - client_ip               │  │
│  │    - query                   │  │
│  │    - user_agent              │  │
│  │    - referer                 │  │
│  └────────────┬─────────────────┘  │
│               │                    │
│               ▼                    │
│  ┌──────────────────────────────┐  │
│  │ 2. Query SQLite FTS5         │──┼──┐
│  │    - Search documents        │  │  │
│  │    - Get result count        │  │  │
│  └────────────┬─────────────────┘  │  │
│               │                    │  │
│               ▼                    │  │
│  ┌──────────────────────────────┐  │  │
│  │ 3. Log to PostgreSQL         │──┼──┼──┐
│  │    - Insert search_logs      │  │  │  │
│  └────────────┬─────────────────┘  │  │  │
│               │                    │  │  │
│               ▼                    │  │  │
│  ┌──────────────────────────────┐  │  │  │
│  │ 4. Build JSON response       │  │  │  │
│  │    - Format results          │  │  │  │
│  │    - Add pagination          │  │  │  │
│  │    - Add execution time      │  │  │  │
│  └────────────┬─────────────────┘  │  │  │
└───────────────┼────────────────────┘  │  │
                │                       │  │
                ▼                       │  │
        ┌───────────────┐               │  │
        │ JSON Response │               │  │
        └───────┬───────┘               │  │
                │                       │  │
                ▼                       │  │
        ┌──────────┐                    │  │
        │  Client  │                    │  │
        └──────────┘                    │  │
                                        │  │
        ┌──────────────────┐            │  │
        │  SQLite DB       │◄───────────┘  │
        │  (search.db)     │               │
        │  - documents_fts │               │
        └──────────────────┘               │
                                           │
        ┌────────────────────┐             │
        │  PostgreSQL DB     │◄────────────┘
        │  (searchlogs)      │
        │  - search_logs     │
        └────────────────────┘
```

---

## Analytics Request Flow

### Popular Searches Endpoint

**Request**:
```http
GET /analytics/popular?days=7&limit=20 HTTP/1.1
```

**Flow**:

1. **FastAPI receives request**
   ```python
   async def get_popular_searches(limit: int = 20, days: int = 7)
   ```

2. **Query PostgreSQL**
   ```sql
   SELECT
       query,
       COUNT(*) as search_count,
       AVG(execution_time) as avg_execution_time,
       AVG(results_count) as avg_results_count
   FROM search_logs
   WHERE timestamp >= NOW() - INTERVAL '7 days'
   GROUP BY query
   ORDER BY search_count DESC
   LIMIT 20;
   ```

3. **Return aggregated data**
   ```json
   {
       "status": "success",
       "period_days": 7,
       "count": 20,
       "popular_searches": [
           {
               "query": "robot",
               "search_count": 145,
               "avg_execution_time": 0.112,
               "avg_results_count": 42
           },
           // ... more results
       ]
   }
   ```

---

## Document Indexing Flow

### Adding New Documents to Search Index

**Trigger**: Content build process (`build_search.py`)

**Flow**:

1. **Read content files**
   ```python
   # From courses
   for course in courses:
       for lesson in course.lessons:
           extract_content(lesson)

   # From blog posts
   for post in blog_posts:
       extract_content(post)
   ```

2. **Insert into SQLite**
   ```python
   insert_document(
       title="MicroPython Basics",
       content="Full lesson content...",
       url="/learn/micropython/01_intro.html",
       cover_image="/assets/cover.jpg",
       page_title="Introduction to MicroPython",
       description="Learn the basics of MicroPython",
       date="2024-01-15",
       author="Kevin McAleer"
   )
   ```

3. **SQLite FTS5 indexing**
   ```sql
   INSERT INTO documents_fts
       (title, content, url, cover_image, page_title, description, date, author)
   VALUES
       (?, ?, ?, ?, ?, ?, ?, ?);
   ```

---

## Error Handling Flow

### Search Query Error

```
Client Request
    ↓
FastAPI Validation
    ↓
┌─────────────────┐
│ Query too short?│
│ Invalid params? │
└────┬────────────┘
     │ YES
     ▼
Return 422 Error
```

### Database Connection Error

```
FastAPI → PostgreSQL Connection
         ↓
    Connection fails
         ↓
┌─────────────────────┐
│ Log error to console│
│ Continue execution  │
│ (graceful degradation)│
└──────────┬──────────┘
           ↓
    Return search results
    (without logging)
```

### Search Timeout

```
SQLite Query → Timeout (>60s)
    ↓
FastAPI catches timeout
    ↓
Return 504 Gateway Timeout
```

---

## Data Lifecycle

### Search Log Data

```
Search Request
    ↓
INSERT into PostgreSQL
    ↓
Stored indefinitely
    ↓
┌───────────────────────┐
│ Used for analytics:   │
│ - Recent searches     │
│ - Popular queries     │
│ - Statistics          │
└───────────────────────┘
    ↓
(Future) Archive after 1 year
    ↓
(Future) Delete after 2 years
```

### Search Index Data

```
Content created/updated
    ↓
Build process runs
    ↓
INSERT/UPDATE in SQLite FTS5
    ↓
Stored permanently
    ↓
Used for all searches
```

---

## Performance Metrics

### Typical Request Timeline

```
0ms     - Request received
5ms     - Routing completed
10ms    - FastAPI validation
15ms    - SQLite query starts
120ms   - SQLite query completes
125ms   - PostgreSQL log starts
135ms   - PostgreSQL log completes
140ms   - Response assembly
145ms   - Response sent

Total: ~145ms
```

### Database Operation Times

| Operation | Typical Time | Notes |
|-----------|--------------|-------|
| SQLite FTS5 search | 50-150ms | Depends on query complexity |
| SQLite result count | 10-30ms | Faster than full search |
| PostgreSQL insert | 5-10ms | Single row insert |
| PostgreSQL analytics | 50-200ms | Aggregation queries |

---

## Caching Strategy (Future)

### Potential Caching Layers

```
Client Request
    ↓
Cloudflare Cache (static assets)
    ↓
Nginx Cache (not currently enabled)
    ↓
Redis Cache (future enhancement)
    │
    ├─► Cache hit → Return cached results
    │
    └─► Cache miss
            ↓
        Query SQLite
            ↓
        Store in Redis (TTL: 5 minutes)
            ↓
        Return results
```

---

## Monitoring Data Flow

### Application Logs

```
Search Request
    ↓
Console Output:
    - "results found: 18"
    - "Search logged to PostgreSQL with ID: 12345"
    ↓
Docker Logs
    ↓
(Future) Centralized Logging System
```

### Metrics Collection (Future)

```
Each Request
    ↓
Prometheus Metrics:
    - search_requests_total
    - search_duration_seconds
    - search_results_count
    ↓
Grafana Dashboard
```
