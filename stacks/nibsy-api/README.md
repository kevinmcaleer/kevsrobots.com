# Nibsy API

FastAPI microservice that powers the Nibsy recommendation widget on
[kevsrobots.com](https://kevsrobots.com). It owns the content catalogue
(courses, posts, videos, robots, reviews, glossary), the click/impression
log, and the precomputed top-N recommendations table read by the widget.

This PR scaffolds the foundation (issue #66) and adds the
`nibsy_recommendations` table from #73a so the generator/reader can land
in follow-up PRs without a schema migration.

## Layout

```
stacks/nibsy-api/
├── nibsy_api/        # FastAPI app
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

To force an ingest run:

```bash
curl -X POST http://127.0.0.1:8200/api/admin/ingest
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
| `GET /health` | implemented | Returns `{status, content_count, recommendation_count}` |
| `POST /api/admin/ingest` | implemented | Triggers a re-ingest from `NIBSY_DATA_DIR` |
| `GET /api/recommendations` | 501 stub | Reader for `nibsy_recommendations` — lands in #67 |
| `GET /api/trending` | 501 stub | #67 / #72 |
| `GET /api/related/{content_id}` | 501 stub | #67 |
| `POST /api/track/click` | 501 stub | #68 |
| `POST /api/track/impression` | 501 stub | #68 |

## Roadmap

| Issue | Status | Summary |
|---|---|---|
| #66 | done (this PR) | FastAPI scaffold, schema, ingest, /health, admin ingest |
| #67 | open | Recommendation read endpoints (`/recommendations`, `/related`, `/trending`) |
| #68 | open | Click & impression tracking endpoints |
| #69 | open | Production ingest pipeline / scheduled refresh |
| #70 | open | Production deployment (docker-stack, swarm, port 8200) |
| #71 | open | Widget integration on kevsrobots.com pages |
| #72 | open | Trending-by-popularity algorithm |
| #73 | open | Precomputed recommendations system (umbrella) |
| #73a | partial (this PR) | Table added; generator + reader land next |
| #73b | open | AI categorisation / smarter signals |

## Conventions

Mirrors `search_app/` where applicable:
* Python 3.12 base image
* `curl -f /health` healthcheck
* No Alembic — `Base.metadata.create_all` on startup
* Image pushed to `192.168.2.1:5000/nibsy-api:latest` (left to #70)
