# KevsRobots Search API - Documentation Overview

## Introduction

The KevsRobots Search API is a FastAPI-based full-text search service with comprehensive query logging to PostgreSQL. This API powers the search functionality on [www.kevsrobots.com](https://www.kevsrobots.com).

**Version**: 2.0.0
**Last Updated**: 2025-11-20

---

## What's New in 2.0

### Major Changes
- **PostgreSQL Logging**: Search queries now logged to PostgreSQL instead of files
- **Analytics Endpoints**: New endpoints for search analytics and statistics
- **Enhanced Monitoring**: Better health checks and error handling
- **Improved Documentation**: Complete architecture, API, and deployment guides

### Migration from 1.0
- File-based logging (`search-logs.log`) → PostgreSQL database
- New environment variables required (see `.env` configuration)
- New analytics features available
- Backward compatible API endpoints

---

## Quick Links

### Documentation
- **[API Documentation](API.md)** - Complete API endpoint reference
- **[Deployment Guide](DEPLOYMENT.md)** - Production deployment instructions
- **[Architecture](../design/architecture.md)** - System architecture overview
- **[Database Schema](../design/database_schema.md)** - Database structure and design
- **[Data Flow](../design/data_flow.md)** - Request/response flow diagrams

### Code
- **Main Application**: `search_app/app.py`
- **Search Logger**: `search_app/search/search_logger.py`
- **Database Module**: `search_app/search/database.py`

---

## Quick Start

### Prerequisites

- Python 3.10+
- PostgreSQL 14+
- Docker (for production deployment)

### Local Development Setup

1. **Clone repository**:
```bash
git clone https://github.com/kevinmcaleer/kevsrobots.com.git
cd kevsrobots.com/search_app
```

2. **Create virtual environment**:
```bash
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

3. **Install dependencies**:
```bash
pip install -r requirements.txt
```

4. **Configure environment**:
```bash
cp .env.example .env
# Edit .env with your database credentials
nano .env
```

5. **Run application**:
```bash
uvicorn app:app --reload --host 0.0.0.0 --port 8000
```

6. **Test the API**:
```bash
# Health check
curl http://localhost:8000/health

# Search test
curl "http://localhost:8000/search/?query=robot"

# View docs
open http://localhost:8000/docs
```

---

## Environment Configuration

### Required Variables

Create a `.env` file with these variables:

```env
# Application Settings
HOST=0.0.0.0
PORT=8000
ENVIRONMENT=development

# PostgreSQL Settings
DB_HOST=192.168.2.3
DB_PORT=5433
DB_NAME=searchlogs
DB_USER=your_username
DB_PASSWORD=your_password

# Optional
ENABLE_ACTIVITY_LOGGING=true
```

**Important**:
- Never commit `.env` to version control
- Use strong passwords for PostgreSQL
- URL-encode special characters in passwords

---

## API Endpoints

### Search
```bash
GET /search/?query=robot&page=1&page_size=10
```
Returns search results with pagination.

### Analytics - Recent Searches
```bash
GET /analytics/recent?limit=100
```
Returns recent search queries.

### Analytics - Popular Searches
```bash
GET /analytics/popular?days=7&limit=20
```
Returns most popular searches in time period.

### Analytics - Statistics
```bash
GET /analytics/stats?days=30
```
Returns aggregate search statistics.

### Health Check
```bash
GET /health
```
Returns API health status.

**See [API.md](API.md) for complete documentation.**

---

## Architecture Overview

### Components

```
┌─────────────┐
│   Client    │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│  FastAPI    │
│     App     │
└──┬────────┬─┘
   │        │
   ▼        ▼
┌────────┐ ┌──────────┐
│ SQLite │ │PostgreSQL│
│ Search │ │  Logging │
└────────┘ └──────────┘
```

### Key Features

1. **Full-Text Search**
   - SQLite FTS5 for fast searching
   - Ranked results (BM25 algorithm)
   - Support for phrase and wildcard searches

2. **Query Logging**
   - All searches logged to PostgreSQL
   - Tracks IP, query, results, timing
   - Indexes for fast analytics

3. **Analytics**
   - Recent searches tracking
   - Popular queries aggregation
   - Performance statistics

4. **Production Ready**
   - Docker containerized
   - Health checks
   - CORS support
   - Error handling

**See [architecture.md](../design/architecture.md) for details.**

---

## Database Schema

### PostgreSQL: search_logs

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

### SQLite: documents_fts

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

**See [database_schema.md](../design/database_schema.md) for complete schema.**

---

## Development Workflow

### 1. Make Changes

Edit code in `search_app/`:
- `app.py` - API endpoints
- `search/search_logger.py` - PostgreSQL logging
- `search/database.py` - SQLite search

### 2. Test Locally

```bash
# Run with auto-reload
uvicorn app:app --reload

# Run tests (if available)
pytest

# Check code style
flake8 app.py
black app.py
```

### 3. Build Docker Image

```bash
docker build -t search-api:latest .
docker run -p 8000:8000 --env-file .env search-api:latest
```

### 4. Deploy to Production

See [DEPLOYMENT.md](DEPLOYMENT.md) for complete deployment instructions.

---

## Testing

### Manual Testing

```bash
# Search endpoint
curl "http://localhost:8000/search/?query=robot"

# Analytics endpoints
curl "http://localhost:8000/analytics/recent?limit=10"
curl "http://localhost:8000/analytics/popular?days=7"
curl "http://localhost:8000/analytics/stats?days=30"

# Health check
curl "http://localhost:8000/health"
```

### Database Verification

```bash
# Check search logs
psql -h 192.168.2.3 -p 5433 -U search_user -d searchlogs

SELECT COUNT(*) FROM search_logs;
SELECT * FROM search_logs ORDER BY timestamp DESC LIMIT 5;
```

---

## Monitoring

### Application Health

```bash
# Check container status
docker ps | grep search-api

# View logs
docker logs -f search-api

# Health endpoint
curl http://localhost:8000/health
```

### Database Health

```bash
# PostgreSQL connection
psql -h 192.168.2.3 -p 5433 -U search_user -d searchlogs -c "SELECT 1;"

# Table size
psql -h 192.168.2.3 -p 5433 -U search_user -d searchlogs -c \
    "SELECT pg_size_pretty(pg_total_relation_size('search_logs'));"

# Recent activity
psql -h 192.168.2.3 -p 5433 -U search_user -d searchlogs -c \
    "SELECT COUNT(*) FROM search_logs WHERE timestamp > NOW() - INTERVAL '1 hour';"
```

---

## Troubleshooting

### Common Issues

**1. Database Connection Failed**
```
Error: psycopg2.OperationalError: could not connect to server
```
**Solution**: Check `.env` file database credentials, ensure PostgreSQL is running and accessible.

**2. Search Returns No Results**
```json
{"results": [], "total_count": 0}
```
**Solution**: Rebuild search index with `build_search.py` or check SQLite database.

**3. Container Unhealthy**
```
Status: unhealthy
```
**Solution**: Check logs (`docker logs search-api`), verify health endpoint, ensure database connectivity.

**See [DEPLOYMENT.md](DEPLOYMENT.md) for detailed troubleshooting.**

---

## Performance

### Benchmarks

| Operation | Typical Time | Notes |
|-----------|--------------|-------|
| Search Query | 50-150ms | Depends on query complexity |
| PostgreSQL Log | 5-10ms | Single row insert |
| Analytics Query | 50-200ms | Aggregation queries |
| Health Check | <5ms | Simple status response |

### Optimization Tips

1. **Database Indexes**: Ensure all indexes exist on `search_logs` table
2. **SQLite Optimization**: Run `VACUUM` and `ANALYZE` periodically
3. **Connection Pooling**: Consider SQLAlchemy pool for PostgreSQL
4. **Caching**: Add Redis for frequently accessed analytics
5. **Pagination**: Use reasonable page sizes (10-50 results)

---

## Security

### Best Practices

- ✅ Database credentials in environment variables
- ✅ PostgreSQL on private network
- ✅ CORS properly configured
- ✅ SSL/TLS via Cloudflare
- ✅ No sensitive data in logs
- ✅ Regular security updates

### Future Enhancements

- [ ] API authentication/authorization
- [ ] Rate limiting per IP
- [ ] SQL injection prevention (using parameterized queries)
- [ ] Input validation and sanitization
- [ ] Audit logging for admin actions

---

## Contributing

### How to Contribute

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Test thoroughly
5. Commit changes (`git commit -m 'Add amazing feature'`)
6. Push to branch (`git push origin feature/amazing-feature`)
7. Open a Pull Request

### Code Style

- Follow PEP 8 for Python code
- Use type hints where possible
- Add docstrings to functions
- Update documentation for new features

---

## Roadmap

### Planned Features

**v2.1** (Q1 2026)
- [ ] User authentication for analytics
- [ ] Rate limiting
- [ ] Export analytics to CSV/JSON
- [ ] Search suggestions/autocomplete

**v2.2** (Q2 2026)
- [ ] Redis caching layer
- [ ] Real-time analytics dashboard
- [ ] Advanced search filters
- [ ] Geographic analytics

**v3.0** (Q3 2026)
- [ ] Machine learning search ranking
- [ ] Multi-language support
- [ ] Search history per user
- [ ] API versioning (v3)

---

## Changelog

### Version 2.0.0 (2025-11-20)

**Added**:
- PostgreSQL logging for search queries
- Analytics endpoints (recent, popular, stats)
- Comprehensive documentation
- Health check endpoint
- Environment-based configuration

**Changed**:
- Migrated from file-based logging to PostgreSQL
- Improved error handling
- Enhanced CORS configuration

**Deprecated**:
- File-based logging (`search-logs.log`)

**Fixed**:
- Pagination calculation edge cases
- Error handling for database failures

### Version 1.0.0 (2024-XX-XX)

**Initial Release**:
- Basic search functionality
- SQLite FTS5 full-text search
- File-based logging
- CORS support

---

## Support

### Getting Help

- **Documentation**: Start here in the `/documentation` folder
- **Issues**: [GitHub Issues](https://github.com/kevinmcaleer/kevsrobots.com/issues)
- **Email**: kevinmcaleer@gmail.com
- **Website**: [www.kevsrobots.com](https://www.kevsrobots.com)

### Reporting Bugs

Please include:
- Description of the issue
- Steps to reproduce
- Expected vs actual behavior
- Environment (Python version, OS, Docker version)
- Relevant logs

---

## License

Copyright © 2024-2025 Kevin McAleer

See LICENSE file for details.

---

## Acknowledgments

- **FastAPI**: Modern Python web framework
- **PostgreSQL**: Robust relational database
- **SQLite FTS5**: Fast full-text search
- **Raspberry Pi**: Affordable hosting platform
- **Cloudflare**: CDN and security services

---

**Built with ❤️ for the maker community**
