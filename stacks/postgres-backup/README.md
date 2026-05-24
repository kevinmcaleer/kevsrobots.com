# postgres-backup

Scheduled `pg_dump` backups of the `kevsrobots_cms` database to an
NFS share on the NAS. Runs as a Docker Swarm service.

## What it does

* Runs forever in a sleep-loop, waking at **00:00 and 12:00 UTC**
  to dump the database via `pg_dump -Fc` (custom format, internally
  compressed).
* Writes to `<nfs share>/daily/` — newest 28 files kept
  (= 14 days at the 12 h cadence).
* On Sundays at 00:00 UTC also copies the dump to
  `<nfs share>/weekly/` — newest 4 weekly snapshots kept.
* Reads DB credentials from Docker swarm secrets
  (`postgres_user`, `postgres_password`) — the same ones the
  postgres service uses, so there's nothing extra to rotate.
* Talks to postgres over the swarm overlay network via the DNS alias
  `postgres_primary` — no host-network punching, no IP hardcoding.

## Before you deploy

### 1. Fill in `.env`

```bash
cp .env.example .env
$EDITOR .env
```

You must set at minimum:

| Variable          | Example                                  |
| ----------------- | ---------------------------------------- |
| `NAS_IP`          | `192.168.2.10`                           |
| `NAS_EXPORT_PATH` | `:/volume1/backups/postgres` (note `:`)  |

The rest have sensible defaults. See `.env.example` for the full
list with explanations.

`.env` is git-ignored — your filled-in copy stays local to the
node you deploy from.

### 2. Confirm the prerequisites

The `deploy.sh` wrapper checks these for you, but it's worth
running them by hand the first time so you understand what
"deployed and healthy" looks like:

```bash
# Secrets exist (created with the postgres stack):
docker secret ls | grep postgres_

# Overlay network exists (default name: postgres_default):
docker network ls | grep postgres

# NFS export is reachable from this node:
showmount -e <NAS_IP>
```

## Deploy

```bash
./deploy.sh
```

The wrapper sources `.env`, sanity-checks secrets + network +
required variables, then runs `docker stack deploy`. If anything's
missing you get a friendly error before anything is created.

The service runs an **immediate first backup on startup** so you
can verify the plumbing without waiting up to 12 hours:

```bash
docker service logs -f postgres-backup_backup
# Expect:
#   [...Z] postgres-backup booting
#   [...Z]   host: postgres_primary:5432  db: kevsrobots_cms
#   [...Z]   storage: daily=/backups/daily (keep 28)
#   [...Z]            weekly=/backups/weekly (keep 4)
#   [...Z] backup start  -> /backups/daily/kevsrobots_cms_…dump
#   [...Z] backup ok     (12M)
#   [...Z] sleeping 21452s until next slot
```

Verify a dump landed on the NAS (from a workstation that has the
share mounted):

```bash
ls -lh <NAS mount>/daily/
# Should show one ${db}_${ts}.dump file.
```

## Why username + password aren't in `.env`

Docker swarm secrets are a strictly better place for credentials
than environment variables:

* Env vars leak into `docker inspect` output and shell history.
  Secrets are mounted as files (`/run/secrets/postgres_user`) and
  never surface in metadata.
* The postgres service already uses these secrets — putting them
  in `.env` would mean two places to rotate them every time the
  password changes.

If the secrets are missing for some reason, recreate via:

```bash
printf 'kev' | docker secret create postgres_user -
printf 'YOUR-PASSWORD' | docker secret create postgres_password -
```

## Restore

The dumps are `pg_dump -Fc` custom format — restore with `pg_restore`:

```bash
# Copy the dump off the NAS to a host that can reach postgres.
scp nas:/volume1/backups/postgres/daily/kevsrobots_cms_2026-05-24_00-00-00.dump .

# Recreate an empty target database first (do NOT restore into the
# live db unless you intend to wipe it):
psql -h 192.168.2.3 -p 5433 -U <user> \
     -c "CREATE DATABASE kevsrobots_cms_restore;"

# Restore. -j 4 = 4-way parallel restore (custom format supports it).
pg_restore \
  -h 192.168.2.3 -p 5433 -U <user> \
  -d kevsrobots_cms_restore \
  -j 4 \
  kevsrobots_cms_2026-05-24_00-00-00.dump

# Inspect, then swap if you're happy:
#   - Stop the projects-api service.
#   - ALTER DATABASE kevsrobots_cms RENAME TO kevsrobots_cms_old;
#   - ALTER DATABASE kevsrobots_cms_restore RENAME TO kevsrobots_cms;
#   - Restart projects-api.
```

To list the contents of a dump without restoring (useful for
sanity-checking schema vs an old version):

```bash
pg_restore -l kevsrobots_cms_2026-05-24_00-00-00.dump | less
```

## Troubleshooting

| Symptom                                            | Likely cause + fix                                                                                                |
| -------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------- |
| `deploy.sh` says `.env not found`                  | Copy `.env.example` to `.env` and fill in.                                                                        |
| `deploy.sh` says `NAS_IP is empty`                 | A required line in `.env` is blank — open the file and fill it in.                                                |
| `deploy.sh` says `docker secret 'postgres_user' missing` | Recreate via the `printf … \| docker secret create` lines above.                                            |
| Service stuck in `Preparing` forever               | NFS mount failing. Check `docker service ps postgres-backup_backup --no-trunc` for the error, then `showmount -e <NAS_IP>` from the host. |
| Logs show `pg_dump: connection to server failed`   | Wrong network. Re-check `POSTGRES_NETWORK` in `.env` — must match an actual overlay (`docker network ls`).        |
| Logs show `pg_dump: error: server version mismatch`| Database is on a newer Postgres than `postgres:16-alpine`. Bump the image tag in `docker-compose.prod.yml` to match. |
| Daily folder filling up past 28 files              | The prune step needs write access to its own files on NFS — confirm `no_root_squash` (or matching uid/gid) on the NFS export. |

## Why this design

* **`.env` for config, secrets for creds.** Two different
  sensitivities deserve two different stores.
* **`deploy.sh` wrapper.** `docker stack deploy` doesn't auto-read
  `.env`; rather than asking everyone to remember the `set -a;
  source .env; set +a` dance, the wrapper does it (plus the
  prerequisite checks). Manual deploy is documented in the
  compose file's header for the day you don't want to use it.
* **Sleep loop, not cron.** Docker Swarm has no native cron, and
  adding a cron daemon means a second supervisor process. A pure
  `sleep` loop respects SIGTERM (trap → graceful exit) and is
  about the same total code.
* **Custom format dumps.** Restore is one `pg_restore` command,
  supports parallel restore, and contents are inspectable via
  `pg_restore -l` without decompressing first.
