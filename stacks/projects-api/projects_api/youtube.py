"""YouTube URL / id extraction (issue #171).

The project editor accepts pastes of any of the common YouTube URL
shapes and stores only the 11-char video id. Centralising the
extractor here keeps the route + tests + (potentially) other callers
in lock-step. Case is preserved exactly — YouTube ids are
case-sensitive (``dQw4w9WgXcQ`` != ``dqw4w9wgxcq``).

Supported inputs:

* ``https://www.youtube.com/watch?v=<id>`` (+ extra query params)
* ``https://youtu.be/<id>`` (+ extra query params)
* ``https://www.youtube.com/embed/<id>``
* ``https://www.youtube.com/shorts/<id>``
* a bare 11-character id

Returns the 11-character id on success. Raises :class:`ValueError`
for anything else (empty string, wrong host, malformed id, missing
``v`` param). The router maps the error to a 422 with a clear
detail message so the frontend can surface "not a YouTube URL".
"""

from __future__ import annotations

import re
from typing import Optional
from urllib.parse import parse_qs, urlparse

# YouTube video ids are exactly 11 characters: letters (case-sensitive),
# digits, hyphen, underscore. Anchoring is essential — a substring match
# on a longer "id-like" token would silently truncate.
_YT_ID_RE = re.compile(r"^[A-Za-z0-9_-]{11}$")

# Hosts we treat as YouTube. Sub-domain stripping is handled below so
# both ``www.youtube.com`` and bare ``youtube.com`` match.
_YT_HOSTS = {"youtube.com", "m.youtube.com", "music.youtube.com"}
_YT_SHORT_HOSTS = {"youtu.be"}


def _is_id(candidate: str) -> bool:
    """Return True iff ``candidate`` matches the 11-char id shape."""
    return bool(_YT_ID_RE.match(candidate))


def _strip_host(host: str) -> str:
    """Lower-case ``host`` and drop a leading ``www.`` for comparison."""
    host = (host or "").lower()
    if host.startswith("www."):
        host = host[4:]
    return host


def extract_youtube_id(url_or_id: str) -> str:
    """Return the 11-char YouTube id for ``url_or_id``.

    The extractor is deliberately strict: anything that doesn't look
    like one of the recognised YouTube shapes raises ``ValueError``.
    The router maps the error to a 422 so the frontend can show a
    clear "not a YouTube URL" message rather than silently saving
    a bad row.
    """
    if url_or_id is None:
        raise ValueError("Empty YouTube URL or id")
    s = str(url_or_id).strip()
    if not s:
        raise ValueError("Empty YouTube URL or id")

    # Bare id (no scheme / slash). Treat as already-extracted.
    if _is_id(s):
        return s

    # Anything else has to parse as a URL. parse_qs / urlparse never
    # raises on weird input — they return empty parts — so we have to
    # check the host + path ourselves.
    parsed = urlparse(s if "://" in s else "https://" + s)
    host = _strip_host(parsed.hostname or "")
    path = parsed.path or ""

    candidate: Optional[str] = None
    if host in _YT_SHORT_HOSTS:
        # https://youtu.be/<id>[?…]
        # The id is the first path segment after the leading slash.
        seg = path.lstrip("/").split("/", 1)[0]
        if seg:
            candidate = seg
    elif host in _YT_HOSTS:
        # /watch?v=<id>  — the canonical form
        if path == "/watch" or path.startswith("/watch/"):
            qs = parse_qs(parsed.query or "")
            vals = qs.get("v") or []
            if vals:
                candidate = vals[0]
        # /embed/<id>
        elif path.startswith("/embed/"):
            candidate = path[len("/embed/"):].split("/", 1)[0]
        # /shorts/<id>
        elif path.startswith("/shorts/"):
            candidate = path[len("/shorts/"):].split("/", 1)[0]
        # /v/<id> (legacy share URL) — cheap to support, easy to forget.
        elif path.startswith("/v/"):
            candidate = path[len("/v/"):].split("/", 1)[0]

    if candidate and _is_id(candidate):
        return candidate

    raise ValueError("Not a recognised YouTube URL or id")
