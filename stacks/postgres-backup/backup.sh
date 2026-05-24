#!/bin/sh
# postgres-backup — runs forever, dumping the kevsrobots_cms database
# to an NFS-mounted /backups directory at 00:00 and 12:00 UTC.
#
# Backup format: pg_dump -Fc (custom format, internally compressed).
# Restore with: pg_restore -h <host> -U <user> -d <db> file.dump
#
# Retention:
#   daily/   — newest 28 files (= 14 days at 12 h cadence)
#   weekly/  — newest 4 files (Sundays 00:00 UTC)
#
# Designed for Docker Swarm: lives in the same overlay network as
# the postgres_primary service, talks to it via swarm DNS, picks up
# DB credentials from /run/secrets/* (same secrets the postgres
# service uses, so they round-trip cleanly).

set -eu

POSTGRES_HOST="${POSTGRES_HOST:-postgres_primary}"
POSTGRES_PORT="${POSTGRES_PORT:-5432}"
POSTGRES_DB="${POSTGRES_DB:-kevsrobots_cms}"
POSTGRES_USER="$(cat /run/secrets/postgres_user)"
PGPASSWORD="$(cat /run/secrets/postgres_password)"
export PGPASSWORD

BACKUP_ROOT="${BACKUP_ROOT:-/backups}"
DAILY_DIR="$BACKUP_ROOT/daily"
WEEKLY_DIR="$BACKUP_ROOT/weekly"
DAILY_KEEP="${DAILY_KEEP:-28}"
WEEKLY_KEEP="${WEEKLY_KEEP:-4}"

mkdir -p "$DAILY_DIR" "$WEEKLY_DIR"

log() {
  echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] $*"
}

# Seconds until the next 00:00 or 12:00 UTC boundary.
# Computed in shell math (10# forces base-10 so "08" doesn't parse
# as octal — historic ash/dash gotcha).
next_slot_seconds() {
  h=$(date -u +%H)
  m=$(date -u +%M)
  s=$(date -u +%S)
  now=$(( 10#$h * 3600 + 10#$m * 60 + 10#$s ))
  if [ "$now" -lt 43200 ]; then
    # Before noon UTC — next slot is today 12:00.
    target=43200
  else
    # After noon UTC — next slot is tomorrow 00:00.
    target=86400
  fi
  echo $(( target - now ))
}

# Single backup + (optional) weekly-snapshot mirror.
# Returns non-zero on dump failure so the caller can log + continue.
run_backup() {
  ts=$(date -u +%Y-%m-%d_%H-%M-%S)
  daily_file="$DAILY_DIR/${POSTGRES_DB}_${ts}.dump"
  log "backup start  -> $daily_file"
  if pg_dump -Fc \
       -h "$POSTGRES_HOST" -p "$POSTGRES_PORT" \
       -U "$POSTGRES_USER" -d "$POSTGRES_DB" \
       -f "$daily_file"; then
    size=$(du -h "$daily_file" | cut -f1)
    log "backup ok     ($size)"
  else
    log "backup FAILED — removing half-written file"
    rm -f "$daily_file"
    return 1
  fi
  # Weekly snapshot: Sunday 00:00 UTC. ``date +%u`` is 1=Mon … 7=Sun.
  dow=$(date -u +%u)
  hh=$(date -u +%H)
  if [ "$dow" = "7" ] && [ "$hh" = "00" ]; then
    weekly_file="$WEEKLY_DIR/${POSTGRES_DB}_${ts}.weekly.dump"
    cp "$daily_file" "$weekly_file"
    log "weekly mirror -> $weekly_file"
  fi
}

# Delete files past the retention cap. ``ls -1t`` lists newest-first;
# ``tail -n +N`` skips the first N-1 lines so we delete from N onward.
# 2>/dev/null on the ls handles the empty-directory case cleanly.
prune() {
  # Daily
  ls -1t "$DAILY_DIR"/*.dump 2>/dev/null \
    | tail -n +$((DAILY_KEEP + 1)) \
    | xargs -r rm -f
  # Weekly
  ls -1t "$WEEKLY_DIR"/*.dump 2>/dev/null \
    | tail -n +$((WEEKLY_KEEP + 1)) \
    | xargs -r rm -f
}

# Catch SIGTERM cleanly so ``docker service rm`` / a stack update
# doesn't leave a half-written backup behind.
graceful_exit() {
  log "received SIGTERM — exiting"
  exit 0
}
trap graceful_exit TERM INT

log "postgres-backup booting"
log "  host: $POSTGRES_HOST:$POSTGRES_PORT  db: $POSTGRES_DB"
log "  storage: daily=$DAILY_DIR (keep $DAILY_KEEP)"
log "           weekly=$WEEKLY_DIR (keep $WEEKLY_KEEP)"

# Initial backup so the service immediately produces a file you can
# eyeball after first deploy, instead of waiting up to 12 h to see
# whether the plumbing actually works.
run_backup || log "initial backup failed (will retry at next slot)"
prune

# Main loop — sleep until the next 00/12 UTC boundary, back up, prune.
while true; do
  delta=$(next_slot_seconds)
  log "sleeping ${delta}s until next slot"
  # ``sleep`` here returns immediately on SIGTERM via the trap.
  sleep "$delta" || true
  run_backup || log "backup failed (continuing)"
  prune
done
