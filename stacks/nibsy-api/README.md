# Nibsy API

FastAPI microservice that powers the Nibsy recommendation widget on
[kevsrobots.com](https://kevsrobots.com). It owns the content catalogue
(courses, posts, videos, robots, reviews, glossary), the click/impression
log, and the precomputed top-N recommendations table read by the widget.

## Layout

```
stacks/nibsy-api/
├── nibsy_api/        # FastAPI app
│   ├── generator.py  # Heuristic top-N scoring (#74)
│   ├── ingest.py     # YAML → nibsy_content
│   └── routers/
├── tests/            # pytest + aiosqlite + fixtures
├── Dockerfile        # python:3.12.1-slim, uvicorn on 8200
├── docker-compose.yml
└── requirements.txt
```

## Running locally

```bash
cd stacks/nibsy-api
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
export NIBSY_DATA_DIR="$(pwd)/../../web/_data"
uvicorn nibsy_api.main:app --reload --port 8200
```

Then hit `http://127.0.0.1:8200/health`.

On first boot the service ingests `NIBSY_DATA_DIR`, generates an initial
recommendation set, and schedules a refresh every
`RECOMMENDATION_REFRESH_DAYS` days (default `14`).

To force a re-ingest or a recommendation refresh:

```bash
curl -X POST http://127.0.0.1:8200/api/admin/ingest
curl -X POST http://127.0.0.1:8200/api/admin/regenerate-recommendations
```

## Running tests

```bash
cd stacks/nibsy-api
pip install -r requirements.txt
pytest
```

Tests use `aiosqlite` (in-memory) and the bundled `tests/fixtures/_data/`
sample YAML — no Postgres or production data needed.

## Building the Docker image

```bash
cd stacks/nibsy-api
docker build -t nibsy-api-local .
```

Or with compose (also brings up a local Postgres on host port 5434):

```bash
docker compose up --build
```

## Endpoints

| Path | Status | Notes |
|---|---|---|
| `GET /health` | implemented | `{status, content_count, recommendation_count}` |
| `GET /api/recommendations` | implemented | Precomputed top-N lookup keyed by `?page=...` |
| `POST /api/admin/ingest` | implemented | Re-ingest from `NIBSY_DATA_DIR` |
| `POST /api/admin/regenerate-recommendations` | implemented | Ad-hoc recompute (also runs on a 14-day schedule) |
| `GET /api/trending` | 501 stub | #67 / #72 |
| `GET /api/related/{content_id}` | 501 stub | #67 |
| `POST /api/track/click` | 501 stub | #68 |
| `POST /api/track/impression` | 501 stub | #68 |

## How recommendations work

Recommendations are **precomputed in batch**, not generated on each
request. A scheduler runs `generate_recommendations` on a 14-day cadence
(and on first boot when `nibsy_recommendations` is empty); each call
recomputes the top-6 picks per source URL and persists them as a JSONB
array. The widget endpoint then becomes a single indexed lookup.

The v0 scorer (`generator.py`) combines three cheap signals:

```
score(source → target) =
    1.0 × jaccard(source.tags, target.tags)        # tag overlap
  + 0.5 × affinity(source.type, target.type)        # content-type matrix
  + 0.3 × recency_bonus(target.date_published)      # newer targets get a bump
```

After scoring, a greedy pick with a soft same-type penalty
(`0.8 ** count_of_same_type_already_chosen`) breaks the obvious
monoculture failure mode where six courses or six videos would dominate.

The scorer is intentionally simple — the AI categorisation pass in
**#75** layers richer topic extraction on top of `tags`, at which point
this same heuristic produces materially better neighbours without any
algorithm change. The `generator_version` column lets consumers invalidate
caches when the scoring evolves.

## Roadmap

| Issue | Status | Summary |
|---|---|---|
| #66 | done | FastAPI scaffold, schema, ingest, `/health`, admin ingest |
| #74 | done | Heuristic recommendations generator + 14-day scheduler + read endpoint |
| #67 | partial | `/api/recommendations` implemented; `/api/related` and `/api/trending` still stubs |
| #68 | open | Click & impression tracking endpoints |
| #69 | open | Production ingest pipeline from live site assets |
| #70 | open | Production deployment (docker-stack, swarm, port 8200) |
| #71 | open | Widget integration on kevsrobots.com pages |
| #72 | open | Trending integration with Chatter analytics |
| #73 | open | Intelligent recommendations (umbrella) |
| #75 | open | AI categorisation pass for richer tags |
| #76 | open | Course pathway / next-course recommendations (pairs with #43) |
| #78 | open | CORS middleware |
| #79 | open | Dockerfile build-deps trim |
| #80 | open | Minor cleanups (compose `version:`, dead reviews check) |

## Conventions

Mirrors `search_app/` where applicable:
* Python 3.12 base image
* `curl -f /health` healthcheck
* No Alembic — `Base.metadata.create_all` on startup
* Image pushed to `192.168.2.1:5000/nibsy-api:latest` (left to #70)
