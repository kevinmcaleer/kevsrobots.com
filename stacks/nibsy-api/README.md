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

Set `CORS_ORIGINS` to a comma-separated list of allowed browser origins
(default: `http://localhost:4000`):

```bash
export CORS_ORIGINS="https://www.kevsrobots.com,http://localhost:4000"
```

For remote ingestion from the live site (#69):

```bash
export SITE_BASE_URL="https://www.kevsrobots.com"  # default
export INGEST_SCHEDULE_HOUR=1                       # UTC hour for daily ingest (default 1, negative to disable)
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

## Production deployment

Build and push the image to the Pi registry:

```bash
cd stacks/nibsy-api
docker build -t 192.168.2.1:5000/nibsy-api:latest .
docker push 192.168.2.1:5000/nibsy-api:latest
```

Deploy to the swarm:

```bash
docker stack deploy -c docker-stack.yml nibsy
```

The stack connects to Postgres at `192.168.2.1:5433/nibsy`. Create
the database first if it doesn't exist:

```bash
psql -h 192.168.2.1 -p 5433 -U nibsy -c "CREATE DATABASE nibsy;"
```

Verify the deployment:

```bash
docker service ls | grep nibsy
curl http://<any-pi-ip>:8200/health
```

DNS: point `nibsy.kevsrobots.com` to the Cloudflare load balancer,
then add an upstream in the nginx config for port 8200.

## Endpoints

| Path | Status | Notes |
|---|---|---|
| `GET /health` | implemented | `{status, content_count, recommendation_count}` |
| `GET /api/recommendations` | implemented | Precomputed top-N lookup keyed by `?page=...` |
| `GET /api/related/{content_id}` | implemented | Tag/affinity-based related content (#67) |
| `GET /api/recommendations/next-course` | implemented | Next course in pathway or related courses (#76) |
| `GET /api/trending` | implemented | Composite trending scores: clicks + page views + YouTube views (#67, #72) |
| `POST /api/track/click` | implemented | Log recommendation clicks (#68) |
| `POST /api/track/impression` | implemented | Log recommendation impressions (#68) |
| `GET /api/analytics/top-clicked` | implemented | Most clicked content by period (#68) |
| `GET /api/analytics/nibsy-stats` | implemented | Dashboard-level stats (#68) |
| `POST /api/admin/ingest` | implemented | Re-ingest from `NIBSY_DATA_DIR` |
| `GET /api/admin/categorise/export` | implemented | Export content for manual AI categorisation (#75) |
| `POST /api/admin/categorise/import` | implemented | Import categorisation results (#75) |
| `POST /api/admin/recompute-trending` | implemented | Recompute trending scores (#72) |
| `POST /api/admin/ingest-remote` | implemented | Re-ingest from live site over HTTP (#69) |
| `POST /api/admin/regenerate-recommendations` | implemented | Ad-hoc recompute (also runs on a 14-day schedule) |

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
| #67 | done | Context-aware recommendations, related, and trending endpoints |
| #68 | done | Click & impression tracking + analytics endpoints |
| #69 | done | Remote ingest from live site + daily 1am scheduler |
| #70 | done | Production deployment (docker-stack, swarm, port 8200) |
| #71 | done | Widget integration on kevsrobots.com pages |
| #72 | done | Trending integration with page views + click data |
| #73 | done | Intelligent recommendations (umbrella) |
| #75 | done | AI categorisation export/import for richer tags |
| #76 | done | Course pathway / next-course recommendations (pairs with #43) |
| #78 | done | CORS middleware |
| #79 | done | Dockerfile build-deps trim |
| #80 | done | Minor cleanups (compose `version:`, dead reviews check) |

## Conventions

Mirrors `search_app/` where applicable:
* Python 3.12 base image
* `curl -f /health` healthcheck
* No Alembic — `Base.metadata.create_all` on startup
* Image pushed to `192.168.2.1:5000/nibsy-api:latest` (left to #70)
