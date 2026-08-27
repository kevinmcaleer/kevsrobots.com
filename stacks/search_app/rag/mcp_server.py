"""MCP server exposing kevsrobots.com search to LLM clients.

Retrieval only. The LLM does its own generation from the chunks we hand back,
so this costs us nothing per query and needs no API key - and the user's
assistant gets grounded answers with real source URLs instead of half-recalled
guesses about Kevin's tutorials.

Mounted into the FastAPI app at /mcp, so it shares the container, the index and
the Cloudflare tunnel with the regular search API.
"""

from mcp.server.mcpserver import MCPServer
from mcp.server.transport_security import TransportSecuritySettings

from .config import BASE_URL, MCP_ALLOWED_HOSTS, MCP_ALLOWED_ORIGINS
from .retrieve import get_page_chunks, search

mcp = MCPServer(
    "kevsrobots",
    title="KevsRobots Search",
    version=__import__("rag", fromlist=["__version__"]).__version__,
    instructions=(
        "Search kevsrobots.com - Kevin McAleer's robotics site. Use "
        "search_kevsrobots to find tutorials, courses, project write-ups, blog "
        "posts and hardware reviews on Raspberry Pi, Pico, MicroPython, 3D "
        "printing and robot builds. Follow up with get_kevsrobots_page to read "
        "a full page. Always cite the returned URLs."
    ),
)

MAX_RESULTS = 20
MAX_CHUNKS_PER_PAGE_READ = 40


_PLACEHOLDERS = {"none", "untagged", ""}


def _value(meta: dict, key: str) -> str:
    """Read a metadata field, treating Chroma placeholders as absent.

    Empty values are stored as the string "none" because Chroma nulls the whole
    metadata dict if it meets a real None - but "none" must never be shown to
    the model as though it were the actual course name.
    """
    value = str(meta.get(key, "") or "").strip()
    return "" if value.lower() in _PLACEHOLDERS else value


def _format_hit(index: int, hit) -> str:
    meta = hit.metadata
    lines = [
        f"[{index}] {hit.label}",
        f"    url: {BASE_URL}/{hit.url}",
        f"    type: {_value(meta, 'page_type') or 'page'}"
        + (f" | course: {_value(meta, 'course')}" if _value(meta, "course") else "")
        + (f" | date: {_value(meta, 'date')[:10]}" if _value(meta, "date") else ""),
    ]
    tags = [t for t in (meta.get("tags") or []) if t.lower() not in _PLACEHOLDERS]
    if tags:
        lines.append(f"    tags: {', '.join(tags[:8])}")
    lines.append(f"    relevance: {hit.score:.0%}" + ("  (exact match)" if hit.exact else ""))
    body = hit.text.split("\n\n", 1)[-1] if "\n\n" in hit.text else hit.text
    lines.append(f"    {' '.join(body.split())[:600]}")
    return "\n".join(lines)


@mcp.tool()
def search_kevsrobots(
    query: str,
    page_type: str | None = None,
    tag: str | None = None,
    course: str | None = None,
    limit: int = 8,
) -> str:
    """Search Kevin McAleer's robotics site (kevsrobots.com) for tutorials,
    courses, projects, blog posts and hardware reviews.

    Covers Raspberry Pi, Raspberry Pi Pico, MicroPython, 3D printing, robot
    builds (SMARS, Scrawly Wally, BurgerBot and others), electronics, Docker
    and home automation. Use this whenever a question is about one of Kevin's
    projects, courses or articles, or when you want a worked example of a
    robotics or MicroPython technique with a citable source.

    Args:
        query: What to search for. A full question works better than a single
            keyword - the index is semantic, so more context retrieves better.
        page_type: Optional filter. One of: lesson, blog, project, review,
            showcase, how_it_works, course_pathway.
        tag: Optional tag filter, e.g. "micropython", "raspberry pi", "3d printing".
        course: Optional course slug filter, e.g. "micropython", "smars", "docker".
        limit: How many results to return (1-20, default 8).

    Returns:
        Ranked excerpts, each with its source URL so you can cite it.
    """
    limit = max(1, min(int(limit or 8), MAX_RESULTS))
    page_types = [page_type] if page_type else None

    try:
        hits = search(query, top_k=limit, page_types=page_types, tag=tag, course=course)
    except Exception as error:
        return f"Search failed: {error}"

    if not hits:
        return (
            f"No results on kevsrobots.com for {query!r}"
            + (f" with the filters you applied" if (page_type or tag or course) else "")
            + ". Try broader wording, or drop the filters."
        )

    header = f"{len(hits)} result(s) from kevsrobots.com for {query!r}:\n"
    return header + "\n\n".join(_format_hit(i, h) for i, h in enumerate(hits, 1))


@mcp.tool()
def get_kevsrobots_page(url: str) -> str:
    """Read the full indexed text of one page on kevsrobots.com.

    Use this after search_kevsrobots when an excerpt looks right but you need
    the whole tutorial - the complete steps, the full code listing, or the
    parts list - rather than the single matching paragraph.

    Args:
        url: The page URL, either full (https://www.kevsrobots.com/learn/...)
            or site-relative (learn/micropython/01_intro.html), exactly as
            returned by search_kevsrobots.

    Returns:
        The page's indexed text in document order.
    """
    relative = url.strip()
    for prefix in (f"{BASE_URL}/", "https://www.kevsrobots.com/", "http://www.kevsrobots.com/"):
        if relative.startswith(prefix):
            relative = relative[len(prefix) :]
            break
    relative = relative.lstrip("/").split("#")[0].split("?")[0]

    try:
        chunks = get_page_chunks(relative)
    except Exception as error:
        return f"Could not read that page: {error}"

    if not chunks:
        return (
            f"No indexed content for {relative!r}. "
            "Check the URL against a search_kevsrobots result - only indexed "
            "content pages are available."
        )

    meta = chunks[0]["metadata"]
    header = [
        f"# {_value(meta, 'title') or relative}",
        f"url: {BASE_URL}/{relative}",
    ]
    if _value(meta, "course"):
        header.append(f"course: {_value(meta, 'course')}")
    if _value(meta, "date"):
        header.append(f"date: {_value(meta, 'date')[:10]}")
    header.append("")

    body = []
    for chunk in chunks[:MAX_CHUNKS_PER_PAGE_READ]:
        # Drop the breadcrumb line - it repeats for every chunk and the
        # heading is already visible in the text below it.
        text = chunk["text"]
        body.append(text.split("\n\n", 1)[-1] if "\n\n" in text else text)

    truncated = ""
    if len(chunks) > MAX_CHUNKS_PER_PAGE_READ:
        truncated = f"\n\n[truncated - page has {len(chunks)} sections]"

    return "\n".join(header) + "\n\n".join(body) + truncated


def http_app():
    """The ASGI app to mount into FastAPI.

    `stateless_http=True` matters here: the site runs on four nodes behind
    Cloudflare, so consecutive requests from one client can land on different
    containers. A session-bound transport would break the moment that happened;
    stateless mode makes every request self-contained.

    `streamable_http_path="/"` because we mount this at /mcp in the parent app -
    leaving the default would serve it at /mcp/mcp.
    """
    return mcp.streamable_http_app(
        streamable_http_path="/",
        stateless_http=True,
        json_response=True,
        transport_security=TransportSecuritySettings(
            enable_dns_rebinding_protection=True,
            allowed_hosts=MCP_ALLOWED_HOSTS,
            allowed_origins=MCP_ALLOWED_ORIGINS,
        ),
    )
