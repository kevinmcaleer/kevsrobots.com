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

COPY . .

ENV PORT=8210
EXPOSE 8210

# SQLite lives on a mounted /data volume so it survives restarts.
VOLUME ["/data"]

HEALTHCHECK --interval=30s --timeout=10s --start-period=10s --retries=3 \
    CMD curl -f http://127.0.0.1:8210/health || exit 1

CMD ["uvicorn", "status_service.main:app", "--host", "0.0.0.0", "--port", "8210"]
