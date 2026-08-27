# KevsRobots Search API Documentation

## Base URL

**Production**: `https://www.kevsrobots.com`
**Development**: `http://localhost:8000`

## Authentication

Currently, no authentication is required for the search endpoint. Analytics endpoints are public but may require authentication in future versions.

---

## Endpoints

### 1. Search Documents

Search the document index using full-text search.

**Endpoint**: `GET /search/`

**Query Parameters**:

| Parameter | Type | Required | Default | Description |
|-----------|------|----------|---------|-------------|
| `query` | string | Yes | - | Search query string |
| `page` | integer | No | 1 | Page number for pagination |
| `page_size` | integer | No | 10 | Number of results per page |

**Example Request**:
```http
GET /search/?query=robot&page=1&page_size=10 HTTP/1.1
Host: www.kevsrobots.com
```

**Example Response**:
```json
{
    "results": [
        {
            "url": "/learn/smars/",
            "cover_image": "/assets/img/courses/smars/cover.jpg",
            "page_title": "Build a SMARS Robot",
            "description": "Learn to build and program a SMARS robot",
            "date": "2024-01-15",
            "author": "Kevin McAleer"
        },
        // ... more results
    ],
    "total_count": 42,
    "total_pages": 5,
    "page": 1,
    "page_size": 10,
    "execution_time": 0.123
}
```

**Response Fields**:

| Field | Type | Description |
|-------|------|-------------|
| `results` | array | Array of search result objects |
| `total_count` | integer | Total number of matching documents |
| `total_pages` | integer | Total number of pages available |
| `page` | integer | Current page number |
| `page_size` | integer | Number of results per page |
| `execution_time` | float | Query execution time in seconds |

**Result Object Fields**:

| Field | Type | Description |
|-------|------|-------------|
| `url` | string | Document URL path |
| `cover_image` | string | Cover image URL |
| `page_title` | string | Document title |
| `description` | string | Document description |
| `date` | string | Publication date (YYYY-MM-DD) |
| `author` | string | Document author |

**Status Codes**:
- `200 OK` - Success
- `422 Unprocessable Entity` - Invalid parameters
- `500 Internal Server Error` - Server error

**cURL Example**:
```bash
curl "https://www.kevsrobots.com/search/?query=micropython&page=1&page_size=10"
```

**JavaScript Example**:
```javascript
fetch('https://www.kevsrobots.com/search/?query=micropython&page=1&page_size=10')
    .then(response => response.json())
    .then(data => {
        console.log(`Found ${data.total_count} results`);
        data.results.forEach(result => {
            console.log(`${result.page_title}: ${result.url}`);
        });
    });
```

**Python Example**:
```python
import requests

response = requests.get('https://www.kevsrobots.com/search/', params={
    'query': 'micropython',
    'page': 1,
    'page_size': 10
})

data = response.json()
print(f"Found {data['total_count']} results")
for result in data['results']:
    print(f"{result['page_title']}: {result['url']}")
```

---

### 2. Create Document (Admin Only)

Add a new document to the search index.

**Endpoint**: `POST /documents/`

**Query Parameters**:

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| `title` | string | Yes | Document title |
| `content` | string | Yes | Full document content |
| `url` | string | Yes | Document URL |

**Example Request**:
```http
POST /documents/?title=Test&content=Sample%20content&url=/test/ HTTP/1.1
Host: www.kevsrobots.com
```

**Example Response**:
```json
{
    "message": "Document created successfully"
}
```

**Status Codes**:
- `200 OK` - Success
- `422 Unprocessable Entity` - Invalid parameters

---

### 3. Recent Searches Analytics

Get recent search queries.

**Endpoint**: `GET /analytics/recent`

**Query Parameters**:

| Parameter | Type | Required | Default | Description |
|-----------|------|----------|---------|-------------|
| `limit` | integer | No | 100 | Maximum number of results |

**Example Request**:
```http
GET /analytics/recent?limit=50 HTTP/1.1
Host: www.kevsrobots.com
```

**Example Response**:
```json
{
    "status": "success",
    "count": 50,
    "searches": [
        {
            "id": 12345,
            "timestamp": "2025-11-20T14:30:15",
            "client_ip": "192.168.1.100",
            "query": "micropython",
            "results_count": 18,
            "execution_time": 0.123,
            "page": 1,
            "page_size": 10,
            "user_agent": "Mozilla/5.0...",
            "referer": "https://www.kevsrobots.com/"
        },
        // ... more searches
    ]
}
```

**Response Fields**:

| Field | Type | Description |
|-------|------|-------------|
| `status` | string | "success" or "error" |
| `count` | integer | Number of results returned |
| `searches` | array | Array of search log objects |

**Search Log Object Fields**:

| Field | Type | Description |
|-------|------|-------------|
| `id` | integer | Log entry ID |
| `timestamp` | string | ISO 8601 timestamp |
| `client_ip` | string | Client IP address |
| `query` | string | Search query |
| `results_count` | integer | Number of results found |
| `execution_time` | float | Query execution time |
| `page` | integer | Page number requested |
| `page_size` | integer | Results per page |
| `user_agent` | string | Browser user agent |
| `referer` | string | HTTP referer |

**Status Codes**:
- `200 OK` - Success
- `500 Internal Server Error` - Database error

**cURL Example**:
```bash
curl "https://www.kevsrobots.com/analytics/recent?limit=20"
```

---

### 4. Popular Searches Analytics

Get the most popular search queries within a time period.

**Endpoint**: `GET /analytics/popular`

**Query Parameters**:

| Parameter | Type | Required | Default | Description |
|-----------|------|----------|---------|-------------|
| `limit` | integer | No | 20 | Maximum number of results |
| `days` | integer | No | 7 | Number of days to look back |

**Example Request**:
```http
GET /analytics/popular?days=7&limit=20 HTTP/1.1
Host: www.kevsrobots.com
```

**Example Response**:
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
        {
            "query": "micropython",
            "search_count": 98,
            "avg_execution_time": 0.089,
            "avg_results_count": 18
        },
        // ... more results
    ]
}
```

**Response Fields**:

| Field | Type | Description |
|-------|------|-------------|
| `status` | string | "success" or "error" |
| `period_days` | integer | Time period analyzed |
| `count` | integer | Number of results returned |
| `popular_searches` | array | Array of popular query objects |

**Popular Query Object Fields**:

| Field | Type | Description |
|-------|------|-------------|
| `query` | string | Search query |
| `search_count` | integer | Number of times searched |
| `avg_execution_time` | float | Average execution time |
| `avg_results_count` | float | Average number of results |

**Status Codes**:
- `200 OK` - Success
- `500 Internal Server Error` - Database error

**cURL Example**:
```bash
curl "https://www.kevsrobots.com/analytics/popular?days=30&limit=10"
```

**Python Example**:
```python
import requests

response = requests.get('https://www.kevsrobots.com/analytics/popular', params={
    'days': 30,
    'limit': 10
})

data = response.json()
print(f"Top searches in the last {data['period_days']} days:")
for search in data['popular_searches']:
    print(f"{search['query']}: {search['search_count']} searches")
```

---

### 5. Search Statistics

Get aggregate statistics about searches.

**Endpoint**: `GET /analytics/stats`

**Query Parameters**:

| Parameter | Type | Required | Default | Description |
|-----------|------|----------|---------|-------------|
| `days` | integer | No | 30 | Number of days to look back |

**Example Request**:
```http
GET /analytics/stats?days=30 HTTP/1.1
Host: www.kevsrobots.com
```

**Example Response**:
```json
{
    "status": "success",
    "period_days": 30,
    "statistics": {
        "total_searches": 4521,
        "unique_ips": 1234,
        "unique_queries": 567,
        "avg_execution_time": 0.115,
        "avg_results_count": 23.5,
        "last_search": "2025-11-20T14:32:45"
    }
}
```

**Statistics Fields**:

| Field | Type | Description |
|-------|------|-------------|
| `total_searches` | integer | Total number of searches |
| `unique_ips` | integer | Number of unique IP addresses |
| `unique_queries` | integer | Number of unique query strings |
| `avg_execution_time` | float | Average query execution time |
| `avg_results_count` | float | Average number of results per search |
| `last_search` | string | Timestamp of most recent search |

**Status Codes**:
- `200 OK` - Success
- `500 Internal Server Error` - Database error

**cURL Example**:
```bash
curl "https://www.kevsrobots.com/analytics/stats?days=7"
```

---

### 6. Health Check

Check if the API is running and healthy.

**Endpoint**: `GET /health`

**Query Parameters**: None

**Example Request**:
```http
GET /health HTTP/1.1
Host: www.kevsrobots.com
```

**Example Response**:
```json
{
    "status": "healthy",
    "service": "KevsRobots Search API",
    "version": "2.0.0",
    "timestamp": "2025-11-20T14:30:00"
}
```

**Response Fields**:

| Field | Type | Description |
|-------|------|-------------|
| `status` | string | "healthy" or "unhealthy" |
| `service` | string | Service name |
| `version` | string | API version |
| `timestamp` | string | Current server time (ISO 8601) |

**Status Codes**:
- `200 OK` - Service is healthy

**cURL Example**:
```bash
curl "https://www.kevsrobots.com/health"
```

---

## CORS Configuration

The API supports Cross-Origin Resource Sharing (CORS) for the following origins:

- `http://0.0.0.0:4000` (local development)
- `https://www.kevsrobots.com` (production)
- `http://www.kevsrobots.com` (production HTTP)

All HTTP methods and headers are allowed for these origins.

---

## Rate Limiting

Currently, there is no rate limiting implemented. This may be added in future versions.

**Recommended**: Clients should implement their own rate limiting to avoid overwhelming the server.

---

## Error Responses

### Validation Error (422)

```json
{
    "detail": [
        {
            "loc": ["query", "query"],
            "msg": "field required",
            "type": "value_error.missing"
        }
    ]
}
```

### Server Error (500)

```json
{
    "status": "error",
    "message": "Database connection failed"
}
```

---

## Search Query Tips

### Simple Search
```
query=robot
```
Returns documents containing "robot"

### Phrase Search
```
query="raspberry pi"
```
Returns documents containing the exact phrase "raspberry pi"

### Multiple Terms (OR)
```
query=robot OR micropython
```
Returns documents containing either "robot" or "micropython"

### Multiple Terms (AND)
```
query=robot micropython
```
Returns documents containing both "robot" and "micropython"

### Wildcard Search
```
query=robo*
```
Returns documents with words starting with "robo" (robot, robots, robotics)

---

## Pagination

### Calculating Page Numbers

```python
total_count = 42
page_size = 10
total_pages = (total_count // page_size) + (1 if total_count % page_size > 0 else 0)
# total_pages = 5
```

### Getting Next Page

```python
current_page = 1
if current_page < total_pages:
    next_page = current_page + 1
    # Fetch: /search/?query=robot&page=2&page_size=10
```

---

## API Versioning

Current version: `2.0.0`

Version history:
- `2.0.0` - PostgreSQL logging, analytics endpoints
- `1.0.0` - Initial release with file-based logging

Future versions will maintain backward compatibility or use URL versioning (e.g., `/v2/search/`)

---

## OpenAPI / Swagger Documentation

Interactive API documentation is available at:

**Development**: `http://localhost:8000/docs`
**Production**: `https://www.kevsrobots.com/docs` (if enabled)

The API automatically generates OpenAPI schema at `/openapi.json`
