# KevsRobots Search API - System Architecture

## Overview

The KevsRobots Search API is a FastAPI-based application that provides full-text search capabilities with comprehensive query logging to PostgreSQL.

## Architecture Diagram

```
┌─────────────────┐
│   Web Browsers  │
│   & Clients     │
└────────┬────────┘
         │
         │ HTTP/HTTPS
         │
    ┌────▼─────┐
    │ Cloudflare│
    │    CDN    │
    └────┬─────┘
         │
         │
    ┌────▼─────────┐
    │ Nginx Load   │
    │  Balancer    │
    └────┬─────────┘
         │
         │ Port 8000
         │
┌────────▼──────────────┐
│   FastAPI Application │
│  (search_app/app.py)  │
├───────────────────────┤
│  Endpoints:           │
│  - /search/           │
│  - /documents/        │
│  - /analytics/*       │
│  - /health            │
└─────┬──────────┬──────┘
      │          │
      │          │
   ┌──▼──────┐  │
   │ SQLite  │  │
   │ Search  │  │
   │   DB    │  │
   │ (FTS5)  │  │
   └─────────┘  │
                │
         ┌──────▼───────────┐
         │   PostgreSQL     │
         │  192.168.2.3     │
         │  Port: 5433      │
         │  DB: searchlogs  │
         │                  │
         │  Tables:         │
         │  - search_logs   │
         └──────────────────┘
```

## Component Breakdown

### 1. FastAPI Application Layer

**File**: `search_app/app.py`

**Responsibilities**:
- Handle HTTP requests
- Route to appropriate handlers
- Execute search queries
- Log searches to PostgreSQL
- Return JSON responses

**Key Features**:
- CORS middleware for cross-origin requests
- Request validation
- Error handling
- Performance timing

### 2. Search Database Module

**File**: `search_app/search/database.py`

**Responsibilities**:
- SQLite FTS5 full-text search
- Document indexing
- Query execution
- Result ranking

**Technology**: SQLite with FTS5 (Full-Text Search)

### 3. Search Logger Module

**File**: `search_app/search/search_logger.py`

**Responsibilities**:
- PostgreSQL connection management
- Search query logging
- Analytics data retrieval
- Table initialization

**Features**:
- Connection pooling
- Error handling
- Environment-based configuration

### 4. PostgreSQL Database

**Location**: `192.168.2.3:5433`
**Database**: `searchlogs`

**Responsibilities**:
- Store search query logs
- Provide analytics data
- Track usage patterns
- Performance monitoring

## Request Flow

### Search Request Flow

1. **Client** sends GET request to `/search/?query=robot&page=1`
2. **Nginx** load balancer routes to available FastAPI instance
3. **FastAPI** receives request:
   - Extracts query parameters
   - Captures client IP and headers
   - Starts timer
4. **Search Database** executes FTS5 query:
   - Returns matching documents
   - Provides result count
5. **Search Logger** writes to PostgreSQL:
   - Logs query details
   - Records execution time
   - Stores metadata
6. **FastAPI** returns JSON response:
   - Search results
   - Pagination info
   - Execution time

### Analytics Request Flow

1. **Client** sends GET request to `/analytics/popular?days=7`
2. **FastAPI** receives request
3. **Search Logger** queries PostgreSQL:
   - Aggregates search data
   - Calculates statistics
4. **FastAPI** returns analytics JSON

## Technology Stack

### Backend
- **Framework**: FastAPI 0.116+
- **Web Server**: Uvicorn
- **Language**: Python 3.10+

### Databases
- **Search**: SQLite with FTS5
- **Logging**: PostgreSQL 14+

### Dependencies
- `psycopg2-binary` - PostgreSQL adapter
- `python-dotenv` - Environment variable management
- `sqlalchemy` - Database toolkit
- `fastapi` - Web framework
- `uvicorn` - ASGI server

### Infrastructure
- **Deployment**: Docker containers
- **Load Balancer**: Nginx
- **CDN**: Cloudflare
- **Platform**: Raspberry Pi 5 cluster (4 nodes)

## Security

### Environment Variables
All sensitive credentials stored in `.env` file:
- Database host
- Database port
- Database name
- Database username
- Database password

### CORS Configuration
Allowed origins:
- `http://0.0.0.0:4000` (local development)
- `https://www.kevsrobots.com` (production)
- `http://www.kevsrobots.com` (production HTTP)

### Network Security
- PostgreSQL on private network (192.168.2.x)
- No public database access
- Cloudflare DDoS protection
- SSL/TLS via Cloudflare

## Scalability Considerations

### Horizontal Scaling
- Multiple FastAPI instances behind load balancer
- Stateless application design
- Shared PostgreSQL database

### Performance Optimization
- SQLite FTS5 for fast full-text search
- Indexed PostgreSQL columns (timestamp, client_ip, query)
- Connection pooling
- Async request handling

### Database Optimization
- Separate read (SQLite) and write (PostgreSQL) concerns
- PostgreSQL indexes on frequently queried columns
- Configurable result pagination

## Monitoring & Logging

### Application Logging
- Console output for debugging
- PostgreSQL for persistent logs
- Structured log format

### Analytics Endpoints
- `/analytics/recent` - Recent searches
- `/analytics/popular` - Popular queries
- `/analytics/stats` - Aggregate statistics

### Health Monitoring
- `/health` endpoint for uptime checks
- Cloudflare health checks
- Docker container health checks

## Deployment Architecture

### Development
```
Local Machine → Docker Compose → localhost:8000
```

### Production
```
GitHub Repo
    ↓
Build Pi → Docker Build → Push to Registry (192.168.2.1:5000)
    ↓
4x Raspberry Pi Nodes → Pull & Deploy
    ↓
Nginx Load Balancer → External Access
    ↓
Cloudflare CDN → Public Internet
```

## Configuration Management

### Environment Variables (.env)
- `DB_HOST` - PostgreSQL host
- `DB_PORT` - PostgreSQL port
- `DB_NAME` - Database name
- `DB_USER` - Database username
- `DB_PASSWORD` - Database password
- `HOST` - Application host
- `PORT` - Application port
- `ENVIRONMENT` - deployment environment

### Runtime Configuration
- CORS origins (hardcoded in app.py)
- Pagination defaults (10 results per page)
- Analytics time windows (configurable via API params)

## Future Enhancements

### Planned Features
1. User authentication for analytics endpoints
2. Rate limiting per IP address
3. Search query caching with Redis
4. Real-time analytics dashboard
5. Export search logs to CSV/JSON
6. Search suggestion/autocomplete
7. Advanced analytics (geographic data, time series)

### Performance Improvements
1. Database connection pooling
2. Async PostgreSQL queries
3. Result caching layer
4. Query optimization

### Monitoring Improvements
1. Prometheus metrics export
2. Grafana dashboards
3. Error tracking (Sentry)
4. Performance monitoring (APM)
