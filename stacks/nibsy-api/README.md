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
├── requirements.txt
└── requirements-dev.txt   # pytest etc., not installed in the image
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

On first boot the service ingests `NIBSY_DATA_DIR` if the content table
is empty. After that the catalogue comes from the live site: the Jekyll
build publishes `/assets/data/nibsy-catalogue.json`
(`web/assets/data/nibsy-catalogue.json`), and the service downloads it
shortly after every start and once a day at `INGEST_SCHEDULE_HOUR`.

To force a refresh (admin login required):

```bash
curl -X POST https://nibsy.kevsrobots.com/api/admin/ingest-remote
```

## Running tests

```bash
cd stacks/nibsy-api
pip install -r requirements-dev.txt
pytest
```

Tests use `aiosqlite` (in-memory) and the bundled `tests/fixtures/_data/`
sample YAML — no Postgres or production data needed.

## Building the Docker image

```bash
cd stacks/nibsy-api
docker build -t nibsy-api-local .
```

## Production deployment

There is one compose file, `docker-compose.yml`, and it is the production
one: `docker compose up -d --build` builds the image (tagged
`192.168.2.1:5000/nibsy-api:latest`) and starts it. To publish the image to
the Pi registry as well, run `docker compose push nibsy-api`.

Create the Postgres database:

```bash
psql -h 192.168.2.1 -p 5433 -U nibsy -c "CREATE DATABASE nibsy;"
```

Copy `.env.example` to `.env` and fill in the tunnel token, then:

```bash
docker compose up -d --build
```

This starts the API and a `cloudflared` sidecar that exposes it via
Cloudflare Tunnel at `nibsy.kevsrobots.com`. Configure the tunnel
in the Cloudflare Zero Trust dashboard to route to
`http://nibsy-api:8200`.

Verify the deployment:

```bash
docker compose ps
curl https://nibsy.kevsrobots.com/health
```

## Endpoints

| Path | Status | Notes |
|---|---|---|
| `GET /health` | implemented | `{status, content_count, recommendation_count}` |
| `GET /api/recommendations` | implemented | Live, page-aware recommendations for `?page=...` |
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
| `POST /api/admin/ingest-remote` | implemented | Re-ingest the site catalogue over HTTP (#69) |
| `POST /api/admin/regenerate-recommendations` | legacy | Rebuilds the precomputed table, which the widget no longer reads |

## How recommendations work

Recommendations are **computed live on every request** (`recommender.py`).
Nothing is stored, so there is nothing to regenerate and nothing that can
go stale; new content is recommendable as soon as the catalogue is
re-ingested.

1. **Resolve the page.** `?page=` is normalised (`/blog/x.html`,
   `/blog/x` and `/blog/x.html?utm=...` are the same page). A lesson
   (`/learn/<course>/<lesson>.html`) uses its course.
2. **Compare it with everything else.** Each catalogue item is a weighted
   bag of terms: tags (x3), title words (x2) and description words (x1),
   each multiplied by IDF so rare terms like `oled` matter more than
   common ones like `python`. Similarity is the cosine between two bags.
   Because titles count, untagged items (YouTube videos, reviews) still
   match.
3. **Break ties** with a small content-type affinity and recency bonus,
   apply a soft same-type penalty so the list is a mix, and keep one slot
   for the best related video.
4. **Fall back** when the page is not in the catalogue (home page,
   section indexes) or has too few related items: trending items for that
   section, rotated daily.

The response carries `basis` (`page`, `course` or `fallback`) so you can
see which path a page took.

The catalogue index is cached in memory and rebuilt only when
`nibsy_content` changes. Scoring ~800 items takes about 2ms.

The older batch generator (`generator.py`, the `nibsy_recommendations`
table and `POST /api/admin/regenerate-recommendations`) is no longer read
by the widget.

### Trending

`trending.py` recomputes scores hourly from Nibsy clicks (7 days), page
views (last 30 days, from the pagecount database at `PAGE_COUNT_URL`),
YouTube views and recency. If `pages_with_views` in the
`/api/admin/recompute-trending` response is 0, the page-view signal is
not connected and trending is recency only.

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

Mirrors `stacks/search_app/` where applicable:
* Python 3.12 base image
* `curl -f /health` healthcheck
* No Alembic — `Base.metadata.create_all` on startup
* Image pushed to `192.168.2.1:5000/nibsy-api:latest` (left to #70)
