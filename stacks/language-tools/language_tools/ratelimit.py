"""A per-user sliding-window rate limit, kept in memory.

One process serves the whole site, so there's nothing to share between
workers; a restart just forgets the counts.
"""

from __future__ import annotations

import time
from collections import defaultdict, deque

from fastapi import HTTPException, status

WINDOW = 60.0


class RateLimiter:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self._hits: dict[tuple[str, str], deque[float]] = defaultdict(deque)

    def hit(self, user: str, bucket: str, limit: int) -> None:
        """Count one request, or raise 429 when ``user`` is over ``limit`` a minute."""
        if limit <= 0:
            return
        now = self.clock()
        hits = self._hits[(user, bucket)]
        while hits and hits[0] <= now - WINDOW:
            hits.popleft()
        if len(hits) >= limit:
            retry = max(1, int(hits[0] + WINDOW - now) + 1)
            raise HTTPException(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                detail="Too many requests. Try again in a moment.",
                headers={"Retry-After": str(retry)},
            )
        hits.append(now)
