# kevsrobots status service. Mirrors nibsy-api / projects-api shape:
# python:3.12-slim, uvicorn on PORT (8210 by default), curl-based
# Docker HEALTHCHECK using 127.0.0.1 (not localhost — see CLAUDE.md
# "Known Issues" re IPv6).

FROM python:3.12-slim

WORKDIR /usr/src/app

RUN apt-get update \
    && apt-get install -y --no-install-recommends curl \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

# Cache-bust the source COPY when main has new commits, so a routine
# rebuild on the Pi picks up code changes without needing --no-cache.
# The atom feed body changes on every commit → ADD re-fetches the remote
# URL on every build → if it changed, this layer + everything after it
# invalidate. Use github.com (NOT api.github.com — rate-limited; see
# CLAUDE.md's "Cache-busting depends on which Dockerfile" note).
ADD https://github.com/kevinmcaleer/kevsrobots.com/commits/main.atom /tmp/cache-bust.xml

COPY . .

ENV PORT=8210
EXPOSE 8210

# SQLite lives on a mounted /data volume so it survives restarts.
VOLUME ["/data"]

HEALTHCHECK --interval=30s --timeout=10s --start-period=10s --retries=3 \
    CMD curl -f http://127.0.0.1:8210/health || exit 1

CMD ["uvicorn", "status_service.main:app", "--host", "0.0.0.0", "--port", "8210"]
