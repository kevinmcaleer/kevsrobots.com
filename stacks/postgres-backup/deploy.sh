#!/bin/sh
# Deploy the postgres-backup stack to Docker Swarm.
#
# Reads variables from .env (which must exist — copy from
# .env.example and fill in NAS_IP / NAS_EXPORT_PATH at minimum),
# exports them, sanity-checks the prerequisites, and runs
# ``docker stack deploy``.
#
# ``docker stack deploy`` does NOT auto-read .env files the way
# ``docker compose up`` does, so this wrapper is the canonical
# way to apply changes from this directory.

set -eu

cd "$(dirname "$0")"

if [ ! -f .env ]; then
  echo "ERROR: .env not found. Copy .env.example to .env first." >&2
  exit 1
fi

# Pull every uncommented assignment out of .env and export it. The
# ``set -a`` toggle marks every variable assigned after it for
# export; ``set +a`` turns it off again so we don't leak shell
# internals.
set -a
# shellcheck disable=SC1091
. ./.env
set +a

# Required variables (a typo'd .env should fail loudly, not deploy
# a broken stack that crash-loops).
for var in NAS_IP NAS_EXPORT_PATH; do
  eval "val=\${$var:-}"
  if [ -z "$val" ]; then
    echo "ERROR: $var is empty in .env — fill it in before deploying." >&2
    exit 1
  fi
done

# Prerequisite: the postgres secrets the container expects must
# already exist on the swarm. They were created with the postgres
# stack, but verify explicitly so the failure is a friendly message
# rather than a cryptic "secret not found" on stack deploy.
for sec in postgres_user postgres_password; do
  if ! docker secret inspect "$sec" >/dev/null 2>&1; then
    echo "ERROR: docker secret '$sec' missing — recreate via:" >&2
    echo "  printf 'kev' | docker secret create postgres_user -" >&2
    echo "  printf 'YOUR-PASSWORD' | docker secret create postgres_password -" >&2
    exit 1
  fi
done

# Prerequisite: the overlay network the backup container joins must
# already exist. Default name is ``postgres_default`` — overridable
# via POSTGRES_NETWORK in .env.
net="${POSTGRES_NETWORK:-postgres_default}"
if ! docker network inspect "$net" >/dev/null 2>&1; then
  echo "ERROR: overlay network '$net' not found." >&2
  echo "Confirm with: docker network ls | grep postgres" >&2
  echo "If your postgres stack uses a different network name, set" >&2
  echo "POSTGRES_NETWORK in .env accordingly." >&2
  exit 1
fi

echo "Deploying postgres-backup stack…"
echo "  NAS:         ${NAS_IP}${NAS_EXPORT_PATH}  (NFSv${NFS_VERSION:-4})"
echo "  Database:    ${POSTGRES_DB:-kevsrobots_cms} @ ${POSTGRES_HOST:-postgres_primary}:${POSTGRES_PORT:-5432}"
echo "  Network:     $net"
echo "  Retention:   daily=${DAILY_KEEP:-28}, weekly=${WEEKLY_KEEP:-4}"
echo

docker stack deploy -c docker-compose.prod.yml postgres-backup

echo
echo "Deployed. Watch the first backup land:"
echo "  docker service logs -f postgres-backup_backup"
