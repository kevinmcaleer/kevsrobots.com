from fastapi import FastAPI, Request, Response
from datetime import datetime
from fastapi.middleware.cors import CORSMiddleware
from search.database import document_count as fts_document_count, insert_document, query_documents, total_results, get_facet_counts
from search.search_logger import get_search_logger
from typing import Optional
import time
import os
from dotenv import load_dotenv

# The RAG engine is optional at runtime: if the vector index failed to build,
# the API must still serve keyword search rather than refusing to start.
try:
    from rag.retrieve import get_page_chunks, search as rag_search
    from rag.vector_index import collection_stats
    RAG_AVAILABLE = True
    RAG_IMPORT_ERROR = None
except Exception as _rag_error:  # pragma: no cover - depends on deploy state
    RAG_AVAILABLE = False
    RAG_IMPORT_ERROR = str(_rag_error)

# Load environment variables
load_dotenv()

app = FastAPI(
    title="KevsRobots Search API",
    description="Search API with PostgreSQL query logging",
    version="3.0.0"
)

# Initialize search logger
search_logger = get_search_logger()

# Mount the MCP server so LLM clients can use the index as a knowledge source.
# Streamable HTTP transport at /mcp, sharing this container's index and tunnel.
MCP_AVAILABLE = False
MCP_IMPORT_ERROR = None
if RAG_AVAILABLE:
    try:
        from contextlib import asynccontextmanager

        from rag.mcp_server import http_app as mcp_http_app

        _mcp_app = mcp_http_app()

        # A mounted sub-app's lifespan is NOT run by the parent, so the MCP
        # session manager would never start. Delegate to it explicitly.
        @asynccontextmanager
        async def _lifespan(_app):
            async with _mcp_app.router.lifespan_context(_mcp_app):
                yield

        app.router.lifespan_context = _lifespan
        app.mount("/mcp", _mcp_app)
        MCP_AVAILABLE = True
    except Exception as _mcp_error:  # pragma: no cover - depends on deploy state
        MCP_IMPORT_ERROR = str(_mcp_error)
        print(f"MCP server not mounted: {_mcp_error}")


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
    allow_origins=[
        "http://0.0.0.0:4000",
        "http://localhost:4000",
        "https://www.kevsrobots.com",
        "http://www.kevsrobots.com",
        "https://beta.kevsrobots.com",
    ],
    allow_credentials=True,
    allow_methods=["*"],  # Allows all methods
    allow_headers=["*"],  # Allows all headers
)

@app.post("/documents/")
def create_document(title: str, content: str, url: str):
    insert_document(title, content, url)
    return {"message": "Document created successfully"}

@app.get("/search/")
async def search_documents(request: Request, query: str, page: Optional[int] = 1, page_size: Optional[int] = 10, sort: Optional[str] = "relevance", page_types: Optional[str] = None, prefix: Optional[bool] = False):
    """
    Search documents endpoint with PostgreSQL query logging.

    Args:
        request: FastAPI request object
        query: Search query string
        page: Page number for pagination (default: 1)
        page_size: Number of results per page (default: 10)
        sort: Sort order — "relevance" (BM25) or "recent" (date descending)
        page_types: Comma-separated page types to filter by (e.g. "lesson,blog")
        prefix: If true, use prefix matching (e.g. "dock" matches "docker")

    Returns:
        dict: Search results with metadata and execution time
    """
    start_time = time.time()

    # Validate sort parameter
    if sort not in ("relevance", "recent"):
        sort = "relevance"

    # Parse page_types filter
    type_filter = [t.strip() for t in page_types.split(',') if t.strip()] if page_types else None

    # Extract real client IP (handles proxy headers)
    client_ip = get_client_ip(request)

    # Extract optional headers
    user_agent = request.headers.get("user-agent")
    referer = request.headers.get("referer")

    # Execute search query
    try:
        results = query_documents(query, offset=(page - 1) * page_size, limit=page_size, sort=sort, page_types=type_filter, prefix=prefix)
        total_count = total_results(query, page_types=type_filter, prefix=prefix)
        facets = get_facet_counts(query)
    except Exception as e:
        print(f"Search query failed: {e}")
        results = []
        total_count = 0
        facets = {}

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
        if log_id is not None:
            print(f'Search logged to PostgreSQL with ID: {log_id}')
    except Exception as e:
        print(f'Failed to log search to PostgreSQL: {e}')

    return {
        "results": results,
        "total_count": total_count,
        "total_pages": (total_count // page_size) + (1 if total_count % page_size > 0 else 0),
        "page": page,
        "page_size": page_size,
        "sort": sort,
        "facets": facets,
        "execution_time": round(execution_time, 3)
    }


@app.get("/search/semantic")
async def semantic_search_endpoint(
    request: Request,
    query: str,
    page: Optional[int] = 1,
    page_size: Optional[int] = 10,
    page_types: Optional[str] = None,
    tag: Optional[str] = None,
    course: Optional[str] = None,
    hybrid: Optional[bool] = True,
):
    """Hybrid semantic + keyword search over the RAG vector index.

    Finds pages by meaning rather than shared words, with exact matches for
    identifier-shaped terms (part numbers, board names) promoted to the top.

    Args:
        query: Search query string.
        page: Page number for pagination (default: 1).
        page_size: Results per page (default: 10).
        page_types: Comma-separated page types, e.g. "lesson,blog".
        tag: Restrict to pages carrying this tag.
        course: Restrict to a course slug, e.g. "micropython".
        hybrid: Include the exact-match arm (default: true).

    Returns:
        dict: Results with metadata, facets and execution time.
    """
    start_time = time.time()

    if not RAG_AVAILABLE:
        return {
            "results": [],
            "total_count": 0,
            "total_pages": 0,
            "page": page,
            "page_size": page_size,
            "engine": "unavailable",
            "error": f"Vector index unavailable: {RAG_IMPORT_ERROR}",
            "execution_time": 0.0,
        }

    type_filter = [t.strip() for t in page_types.split(",") if t.strip()] if page_types else None

    # Retrieve a fixed pool rather than exactly one page's worth, so that
    # total_count and the facet counts describe the whole result set instead
    # of just the slice the caller happened to ask for.
    wanted = max(1, int(page)) * max(1, int(page_size))
    pool = max(wanted, 50)
    try:
        hits = rag_search(
            query,
            top_k=min(pool, 200),
            page_types=type_filter,
            tag=tag,
            course=course,
            hybrid=bool(hybrid),
        )
    except Exception as error:
        print(f"Semantic search failed: {error}")
        return {
            "results": [],
            "total_count": 0,
            "total_pages": 0,
            "page": page,
            "page_size": page_size,
            "engine": "semantic",
            "error": str(error),
            "execution_time": round(time.time() - start_time, 3),
        }

    offset = (max(1, int(page)) - 1) * int(page_size)
    window = hits[offset : offset + int(page_size)]

    facets: dict = {}
    for hit in hits:
        page_type = hit.metadata.get("page_type", "page")
        facets[page_type] = facets.get(page_type, 0) + 1

    execution_time = time.time() - start_time

    try:
        search_logger.log_search(
            client_ip=get_client_ip(request),
            query=query,
            results_count=len(hits),
            execution_time=execution_time,
            page=page,
            page_size=page_size,
            user_agent=request.headers.get("user-agent"),
            referer=request.headers.get("referer"),
        )
    except Exception as error:
        print(f"Failed to log semantic search: {error}")

    return {
        "results": [hit.as_result() for hit in window],
        "total_count": len(hits),
        "total_pages": (len(hits) + int(page_size) - 1) // int(page_size),
        "page": page,
        "page_size": page_size,
        "engine": "hybrid" if hybrid else "semantic",
        "facets": facets,
        "execution_time": round(execution_time, 3),
    }


@app.get("/page/")
async def page_content(url: str):
    """Return every indexed chunk for one page, in document order.

    Lets a client read a whole page after a search surfaces one paragraph.
    """
    if not RAG_AVAILABLE:
        return {"url": url, "chunks": [], "error": "Vector index unavailable"}
    try:
        chunks = get_page_chunks(url)
    except Exception as error:
        return {"url": url, "chunks": [], "error": str(error)}
    return {
        "url": url,
        "chunk_count": len(chunks),
        "title": chunks[0]["metadata"].get("title", "") if chunks else "",
        "chunks": [c["text"] for c in chunks],
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
async def health_check(response: Response):
    """Health check endpoint.

    Reports unhealthy (503) when an index is empty. A container serving an
    empty index answers every query with "no results", which looks like a
    working service but is a total outage from the user's point of view - so
    the healthcheck has to catch it rather than just proving the process is up.
    """
    status = "healthy"
    problems = []

    try:
        fts_count = fts_document_count()
    except Exception as error:
        fts_count = 0
        problems.append(f"FTS index unreadable: {error}")
    if fts_count == 0:
        problems.append("FTS index is empty")

    vector_count = 0
    if RAG_AVAILABLE:
        try:
            vector_count = collection_stats().get("chunks", 0)
        except Exception as error:
            problems.append(f"Vector index unreadable: {error}")
        if vector_count == 0:
            problems.append("Vector index is empty")
    else:
        problems.append(f"RAG engine unavailable: {RAG_IMPORT_ERROR}")

    if problems:
        status = "unhealthy"
        response.status_code = 503

    return {
        "status": status,
        "service": "KevsRobots Search API",
        "version": "3.0.0",
        "fts_documents": fts_count,
        "vector_chunks": vector_count,
        "mcp": "mounted" if MCP_AVAILABLE else f"unavailable: {MCP_IMPORT_ERROR}",
        "problems": problems,
        "timestamp": datetime.now().isoformat(),
    }