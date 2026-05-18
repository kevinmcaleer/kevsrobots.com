# Chatter: `/api/users/{username}/comments_count` endpoint (issue #147)

This is a ready-to-apply spec for the Chatter-side endpoint that unblocks
the six `engaged_member_*` badges in projects-api. The projects-api side
already ships in PR #146 — once Chatter exposes the route below and
projects-api gets `CHATTER_USER_COMMENTS_ENDPOINT` set, the badges start
awarding automatically within the 5-minute cache TTL.

Chatter lives in a separate repo from this one, so this file is a
hand-off doc rather than a code change. Drop the relevant snippet into
Chatter, deploy, then set the env var on projects-api.

---

## Contract

```
GET /api/users/{username}/comments_count

200 OK
Content-Type: application/json
{ "count": 42 }
```

| condition                          | response                                    |
| ---------------------------------- | ------------------------------------------- |
| user exists, any comment count     | `200` with `{ "count": <int> }`             |
| user exists, zero comments         | `200` with `{ "count": 0 }`                 |
| user does not exist                | `200` with `{ "count": 0 }` *(see note)*    |
| internal error                     | `500` — projects-api caches stale on this   |

**Note on unknown users.** projects-api treats any non-200 as a fetch
failure and falls back to the previously-cached value. Returning `200`
with `{ "count": 0 }` for an unknown username is the cleanest fit
because (a) usernames in projects-api come from JWTs that Chatter itself
issued, so the "unknown user" branch should never fire in production,
and (b) it sidesteps having to distinguish 404-because-unknown from
404-because-route-missing on the client side. `404` is also acceptable.

### Auth

Public. The count is non-sensitive (a tally already implicitly visible
to anyone who could scrape the comments). No JWT required.

### Caching

projects-api already wraps this call in a 5-minute TTL cache (see
`stacks/projects-api/projects_api/badge_counter_cache.py`). A
single-digit-millisecond response from Chatter is fine; you don't need
to add caching on the Chatter side unless you see read pressure.

---

## Implementation — FastAPI (most likely Chatter's stack)

Add to whichever module currently mounts `/api/users/*` or `/api/me`
(based on patterns in this repo, Chatter is FastAPI + SQLAlchemy async).

```python
# chatter/routers/users.py  (or wherever user-scoped routes live)

from fastapi import APIRouter, Depends
from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession

from chatter.db import get_session  # adjust import to your project
from chatter.models import Comment  # adjust import to your project

router = APIRouter(prefix="/api/users", tags=["users"])


@router.get("/{username}/comments_count")
async def get_user_comments_count(
    username: str,
    session: AsyncSession = Depends(get_session),
) -> dict[str, int]:
    """Return the total number of comments posted by ``username``.

    Public. Returns 0 for unknown users — see contract note.
    """
    stmt = select(func.count()).select_from(Comment).where(
        Comment.username == username,
    )
    count = (await session.execute(stmt)).scalar_one()
    return {"count": int(count or 0)}
```

**Index.** If `comments.username` isn't already indexed, add one — a
sequential scan over a large comments table will dominate the response
time:

```sql
CREATE INDEX IF NOT EXISTS ix_comments_username ON comments (username);
```

(Use whichever migration tool Chatter uses — Alembic, raw SQL on
startup, etc.)

---

## Implementation — Flask (if Chatter isn't FastAPI)

```python
# chatter/blueprints/users.py

from flask import Blueprint, jsonify
from chatter.db import db
from chatter.models import Comment

users_bp = Blueprint("users", __name__, url_prefix="/api/users")


@users_bp.get("/<username>/comments_count")
def comments_count(username: str):
    count = db.session.query(Comment).filter(
        Comment.username == username
    ).count()
    return jsonify(count=int(count))
```

---

## Implementation — Django (less likely)

```python
# chatter/views/users.py

from django.http import JsonResponse
from chatter.models import Comment


def comments_count(request, username: str):
    count = Comment.objects.filter(username=username).count()
    return JsonResponse({"count": count})


# urls.py
# path("api/users/<str:username>/comments_count", comments_count),
```

---

## Tests (FastAPI variant)

```python
# tests/test_user_comments_count.py

import pytest
from httpx import AsyncClient


@pytest.mark.asyncio
async def test_unknown_user_returns_zero(client: AsyncClient):
    r = await client.get("/api/users/no_such_user/comments_count")
    assert r.status_code == 200
    assert r.json() == {"count": 0}


@pytest.mark.asyncio
async def test_count_matches_seeded_comments(client: AsyncClient, db_session):
    # Seed: alice has 3 comments, bob has 1, charlie has 0.
    for _ in range(3):
        db_session.add(Comment(username="alice", body="hi"))
    db_session.add(Comment(username="bob", body="hi"))
    await db_session.commit()

    r = await client.get("/api/users/alice/comments_count")
    assert r.json() == {"count": 3}

    r = await client.get("/api/users/bob/comments_count")
    assert r.json() == {"count": 1}

    r = await client.get("/api/users/charlie/comments_count")
    assert r.json() == {"count": 0}


@pytest.mark.asyncio
async def test_no_auth_required(client: AsyncClient):
    # No Authorization header, no cookie — should still 200.
    r = await client.get("/api/users/alice/comments_count")
    assert r.status_code == 200
```

---

## Deploy checklist

1. [ ] Add route + index in Chatter.
2. [ ] Deploy Chatter.
3. [ ] Verify against production:
   ```
   curl https://chatter.kevsrobots.com/api/users/kev/comments_count
   ```
   Expect `200` with `{ "count": <int> }`.
4. [ ] Set env var on projects-api:
   ```
   CHATTER_USER_COMMENTS_ENDPOINT=/api/users/{username}/comments_count
   ```
5. [ ] Restart projects-api (the value is read at startup via Pydantic
   settings). The badge counter cache will warm on the next
   `evaluate_user` call per user; first awards arrive within ≤5 minutes.
6. [ ] Spot-check: pick a known-prolific commenter and confirm they earn
   `engaged_member_bronze` (≥10 comments) within the next eval cycle.

## Rollback

If the endpoint misbehaves, unset `CHATTER_USER_COMMENTS_ENDPOINT` on
projects-api and restart. The badge evaluator falls back to the
dormant-returns-0 path with no other side effects. Cached counter
values from the endpoint era stay in memory for up to 5 minutes; they
expire naturally or can be cleared by restarting projects-api.

Refs: #106, #142, #146 (projects-api side), #147 (this work).
