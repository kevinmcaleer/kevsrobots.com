# kevsrobots status service

Public status page + API for every kevsrobots web service. Lives at
**status.kevsrobots.com**. Implements [issue #203](https://github.com/kevinmcaleer/kevsrobots.com/issues/203).

## What it does

* Polls every monitored service's `/health` endpoint every 15 minutes.
* Stores each result in a tiny SQLite DB at `/data/status.db`.
* Vacuums rows older than 30 days once a day at 03:30 UTC.
* Serves a public read-only API:
  * `GET /api/status` — overall + per-service current state
    (`green`/`amber`/`red`)
  * `GET /api/status/{service}` — one service
  * `GET /api/incidents?days=30` — derived incident windows
  * `GET /api/uptime?days=30` — uptime % per service
* Serves a public dashboard at `/` (no auth) styled after
  [status.claude.ai](https://status.claude.ai) — a coloured banner plus
  a 30-day timeline strip per service.

## Layout

```
stacks/status/
├── Dockerfile
├── docker-compose.prod.yml   # compose deploy on a single Pi
├── docker-stack.yml          # Swarm deploy across the cluster
├── .env.example              # every env var documented inline
├── requirements.txt
├── status_service/
│   ├── main.py               # FastAPI app + lifespan + routes
│   ├── config.py             # pydantic-settings
│   ├── db.py                 # aiosqlite schema + helpers
│   ├── polling.py            # probe + classify + write
│   ├── incidents.py          # derive incidents + uptime
│   ├── templates/index.html
│   └── static/dashboard.css
└── tests/                    # pytest, fully hermetic, no real HTTP
```

## Running locally

```bash
cd stacks/status
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

# Override the DB path to anywhere writable in dev.
export STATUS_DB_PATH=./status.db
# (Optional) shrink the service list while testing.
export STATUS_SERVICES="self=http://127.0.0.1:8210/health"

uvicorn status_service.main:app --reload --port 8210
```

Then open <http://127.0.0.1:8210/>.

## Running tests

Tests are fully hermetic — they monkeypatch `httpx.AsyncClient.get` and
never touch the network. The APScheduler scheduler is suppressed by the
conftest setting `STATUS_DISABLE_SCHEDULER=1`.

```bash
cd stacks/status
pip install -r requirements.txt
pytest -q
```

## Building the Docker image

```bash
cd stacks/status
docker build -t 192.168.2.1:5000/status:latest .
docker push 192.168.2.1:5000/status:latest
```

(Built on a Pi or with `--platform linux/arm64` from a Mac, per the
project's `CLAUDE.md`.)

## Production deployment — compose

```bash
cd stacks/status
cp .env.example .env          # then fill in STATUS_TUNNEL_TOKEN, etc.
mkdir -p data                 # SQLite lives here
docker compose -f docker-compose.prod.yml up -d
```

Verify:

```bash
docker compose -f docker-compose.prod.yml ps
curl http://127.0.0.1:8210/health         # local
curl https://status.kevsrobots.com/health # via Cloudflare tunnel
```

## Production deployment — Swarm

`docker stack deploy` does **not** read `.env` automatically. Source it
into the shell first:

```bash
cd stacks/status
cp .env.example .env          # then fill in values
set -a; source .env; set +a   # IMPORTANT — Swarm doesn't read .env
docker stack deploy --compose-file docker-stack.yml status
```

## Configuring the Cloudflare tunnel

In Cloudflare Zero Trust → Tunnels:

1. Create a tunnel called `status`.
2. Copy its token into `STATUS_TUNNEL_TOKEN` in `.env`.
3. Add a public hostname:
   * Subdomain: `status`
   * Domain: `kevsrobots.com`
   * Service: `http://localhost:8210` (the API uses `network_mode: host`
     in the compose file, so `localhost` from the cloudflared sidecar
     reaches the API directly).

## What's NOT exposed

The dashboard is fully public. Only safe data appears in the API:

* Service display names (`search`, `chatter`, …) — already public.
* Current status pill, latency, last-checked time.
* Derived incident windows (start / end / peak severity).
* Uptime percentages.

The internal service URLs configured via `STATUS_SERVICES` are **not**
returned by any endpoint. No usernames, no tokens, no internal IPs.

## Conventions

Mirrors `stacks/nibsy-api/` and `stacks/projects-api/`:

* Python 3.12 base image.
* `curl -f /health` Docker healthcheck on `127.0.0.1` (not `localhost`
  — IPv6/IPv4 footgun per `CLAUDE.md`).
* Image pushed to `192.168.2.1:5000/status:latest`.
* `cloudflared` sidecar with `--edge-ip-version 4` (post-power-cut
  IPv6 footgun, per `CLAUDE.md`).
* APScheduler guarded by `STATUS_DISABLE_SCHEDULER=1` and
  `PYTEST_CURRENT_TEST` so the test suite stays hermetic.
