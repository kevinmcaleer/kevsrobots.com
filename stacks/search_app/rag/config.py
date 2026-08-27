"""All the knobs in one place.

Every tunable in the RAG pipeline lives here so an experiment is a one-line
change rather than a hunt through five modules.
"""

import os
from pathlib import Path

# Where things live -----------------------------------------------------------

# The built Jekyll site. Relative to this package's parent (stacks/search_app/).
_HERE = Path(__file__).resolve().parent.parent
SITE_ROOT = Path(os.environ.get("SITE_ROOT", _HERE / ".." / ".." / "web" / "_site")).resolve()

# Jekyll's _data directory, for metadata enrichment.
DATA_ROOT = Path(os.environ.get("DATA_ROOT", _HERE / ".." / ".." / "web" / "_data")).resolve()

# Where the Chroma vector index is stored. Baked into the image at build time.
CHROMA_PATH = Path(os.environ.get("CHROMA_PATH", _HERE / "chroma")).resolve()

COLLECTION_NAME = "kevsrobots"

# The canonical site URL, for turning relative paths absolute.
BASE_URL = os.environ.get("BASE_URL", "https://www.kevsrobots.com")

# Chunking, in characters -----------------------------------------------------

CHUNK_SIZE = 1200
CHUNK_OVERLAP = 200
MIN_CHUNK_SIZE = 80

# Headings we split on. h5/h6 are usually inline emphasis rather than structure.
HEADING_TAGS = ("h1", "h2", "h3", "h4")

# Prepend the course name to each chunk's embedded text as well as the
# breadcrumb. A lesson on the Q-learning update rule never says the words
# "reinforcement learning", so without this a query paraphrasing the course
# subject cannot reach its own lessons. Measured with eval/evaluate.py.
INCLUDE_COURSE_IN_TEXT = os.environ.get("INCLUDE_COURSE_IN_TEXT", "1") != "0"

# Retrieval -------------------------------------------------------------------

TOP_K = 10
MAX_CHUNKS_PER_PAGE = 2
OVERSAMPLE = 3

# Keep hits at least this fraction as good as the best one. None disables it.
STRAGGLER_RATIO = 0.45

# Content selection -----------------------------------------------------------

# Utility and infrastructure pages that pollute results.
EXCLUDED_PAGE_TYPES = {
    "content",
    "default",
    "home",
    "blog_index",
    "store",
    "tag",
    "groups",
    "course_pathways",
}

# Directories under _site we never index.
SKIP_DIRS = {"assets", "cdn", "node_modules", ".git", "images", "img"}

# HTML elements whose text is navigation or boilerplate, not content.
STRIP_SELECTORS = (
    "nav",
    "header",
    "footer",
    "script",
    "style",
    "noscript",
    "form",
    ".navbar",
    ".sidebar",
    ".breadcrumb",
    ".pagination",
    ".footer",
    "#footer",
    ".cookie-banner",
    ".skip-link",
    ".learn-nav",       # the course lesson sidebar on /learn pages
    ".progress",        # "30% Percent Complete" progress bars
    ".related",
    ".comments",
    "#disqus_thread",
)

# Where a page's real content lives, best guess first. The first selector that
# matches is used; if none match we fall back to <body>. Jekyll's layouts do not
# use <main>, so these are the Bootstrap column classes the layouts actually emit.
CONTENT_SELECTORS = (
    "main",
    "article",
    "div.col-lg-9",
    "div.col-md-9",
    "div.content",
    "div.container-fluid",
)


# MCP transport ---------------------------------------------------------------

# Host headers the MCP endpoint will accept. DNS-rebinding protection rejects
# anything else with a 421, so the production hostname must be listed here.
# Override with MCP_ALLOWED_HOSTS as a comma-separated list.
MCP_ALLOWED_HOSTS = [
    h.strip()
    for h in os.environ.get(
        "MCP_ALLOWED_HOSTS",
        "search.kevsrobots.com,www.kevsrobots.com,localhost,localhost:8000,"
        "127.0.0.1,127.0.0.1:8000,testserver",
    ).split(",")
    if h.strip()
]

MCP_ALLOWED_ORIGINS = [
    o.strip()
    for o in os.environ.get(
        "MCP_ALLOWED_ORIGINS",
        "https://search.kevsrobots.com,https://www.kevsrobots.com,https://claude.ai",
    ).split(",")
    if o.strip()
]
