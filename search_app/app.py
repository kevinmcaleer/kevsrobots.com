from fastapi import FastAPI, Request
from datetime import datetime
from fastapi.middleware.cors import CORSMiddleware
from search.database import insert_document, query_documents, total_results
from search.search_logger import get_search_logger
from typing import Optional
import time
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

app = FastAPI(
    title="KevsRobots Search API",
    description="Search API with PostgreSQL query logging",
    version="2.0.0"
)

# Initialize search logger
search_logger = get_search_logger()


def get_client_ip(request: Request) -> str:
    """
    Extract the real client IP address from request headers.

    When behind proxies (Cloudflare, Nginx), the direct client IP is the proxy.
    The real client IP is in X-Forwarded-For or X-Real-IP headers.

    Priority order:
    1. X-Forwarded-For (first IP in the chain)
    2. X-Real-IP
    3. request.client.host (fallback)

    Args:
        request: FastAPI request object

    Returns:
        str: Client IP address
    """
    # X-Forwarded-For contains a chain of IPs: "client, proxy1, proxy2"
    # The first IP is the original client
    forwarded_for = request.headers.get("x-forwarded-for")
    if forwarded_for:
        # Take the first IP (original client)
        client_ip = forwarded_for.split(",")[0].strip()
        return client_ip

    # X-Real-IP is set by Nginx
    real_ip = request.headers.get("x-real-ip")
    if real_ip:
        return real_ip

    # Fallback to direct connection IP (will be proxy IP)
    return request.client.host if request.client else "unknown"


# Set up CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://0.0.0.0:4000", "https://www.kevsrobots.com", "http://www.kevsrobots.com"],  # List of allowed origins
    allow_credentials=True,
    allow_methods=["*"],  # Allows all methods
    allow_headers=["*"],  # Allows all headers
)

@app.post("/documents/")
def create_document(title: str, content: str, url: str):
    insert_document(title, content, url)
    return {"message": "Document created successfully"}

@app.get("/search/")
async def search_documents(request: Request, query: str, page: Optional[int] = 1, page_size: Optional[int] = 10):
    """
    Search documents endpoint with PostgreSQL query logging.

    Args:
        request: FastAPI request object
        query: Search query string
        page: Page number for pagination (default: 1)
        page_size: Number of results per page (default: 10)

    Returns:
        dict: Search results with metadata and execution time
    """
    start_time = time.time()

    # Extract real client IP (handles proxy headers)
    client_ip = get_client_ip(request)

    # Extract optional headers
    user_agent = request.headers.get("user-agent")
    referer = request.headers.get("referer")

    # Execute search query
    results = query_documents(query, offset=(page - 1) * page_size, limit=page_size)
    total_count = total_results(query)
    execution_time = time.time() - start_time

    # Log to PostgreSQL database
    try:
        log_id = search_logger.log_search(
            client_ip=client_ip,
            query=query,
            results_count=total_count,
            execution_time=execution_time,
            page=page,
            page_size=page_size,
            user_agent=user_agent,
            referer=referer
        )
        print(f'Search logged to PostgreSQL with ID: {log_id}')
    except Exception as e:
        print(f'Failed to log search to PostgreSQL: {e}')
        # Continue even if logging fails - don't break the search functionality

    return {
        "results": results,
        "total_count": total_count,
        "total_pages": (total_count // page_size) + (1 if total_count % page_size > 0 else 0),
        "page": page,
        "page_size": page_size,
        "execution_time": round(execution_time, 3)
    }


@app.get("/analytics/recent")
async def get_recent_searches(limit: Optional[int] = 100):
    """
    Get recent search queries.

    Args:
        limit: Maximum number of results to return (default: 100)

    Returns:
        dict: Recent search log entries
    """
    try:
        searches = search_logger.get_recent_searches(limit=limit)
        return {
            "status": "success",
            "count": len(searches),
            "searches": searches
        }
    except Exception as e:
        return {
            "status": "error",
            "message": str(e)
        }


@app.get("/analytics/popular")
async def get_popular_searches(limit: Optional[int] = 20, days: Optional[int] = 7):
    """
    Get the most popular search queries.

    Args:
        limit: Maximum number of results to return (default: 20)
        days: Number of days to look back (default: 7)

    Returns:
        dict: Popular search queries with counts
    """
    try:
        searches = search_logger.get_popular_searches(limit=limit, days=days)
        return {
            "status": "success",
            "period_days": days,
            "count": len(searches),
            "popular_searches": searches
        }
    except Exception as e:
        return {
            "status": "error",
            "message": str(e)
        }


@app.get("/analytics/stats")
async def get_search_statistics(days: Optional[int] = 30):
    """
    Get aggregate search statistics.

    Args:
        days: Number of days to look back (default: 30)

    Returns:
        dict: Aggregate statistics about searches
    """
    try:
        stats = search_logger.get_search_stats(days=days)
        return {
            "status": "success",
            "period_days": days,
            "statistics": stats
        }
    except Exception as e:
        return {
            "status": "error",
            "message": str(e)
        }


@app.get("/health")
async def health_check():
    """
    Health check endpoint.

    Returns:
        dict: Application health status
    """
    return {
        "status": "healthy",
        "service": "KevsRobots Search API",
        "version": "2.0.0",
        "timestamp": datetime.now().isoformat()
    }