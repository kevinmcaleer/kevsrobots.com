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
* Reads DB credentials from the same Docker secrets the postgres
  service uses (`postgres_user`, `postgres_password`).
* Talks to postgres over the swarm overlay network via the DNS alias
  `postgres_primary` — no host-network punching, no IP hardcoding.

## Before you deploy

Edit `docker-compose.prod.yml` and set:

| Placeholder       | What to put there                                              |
| ----------------- | -------------------------------------------------------------- |
| `NAS_IP`          | LAN IP of the NAS, e.g. `192.168.2.10`                         |
| `NAS_EXPORT_PATH` | NFS export path, e.g. `:/volume1/backups/postgres` (note `:`)  |

Confirm the overlay network name is right — the file defaults to
`postgres_default`, which is what `docker stack deploy postgres ...`
produces:

```bash
docker network ls | grep postgres
# Look for the overlay network created by the postgres stack.
# If it's not "postgres_default", update the networks: block.
```

Confirm the two secrets exist (they were created with the postgres
stack — should already be there):

```bash
docker secret ls | grep postgres_
# Expect: postgres_user, postgres_password.
```

Confirm the NFS export is reachable from every node that might host
the service (any node in the swarm, since there's no placement
constraint):

```bash
# From any swarm node:
showmount -e NAS_IP
# Expect to see your export path listed and not access-controlled
# to a node IP that excludes the current host.
```

## Deploy

```bash
cd ~/ClusteredPi/stacks/postgres-backup  # or wherever you keep it
sudo docker stack deploy -c docker-compose.prod.yml postgres-backup
```

The service runs an **immediate first backup on startup** so you can
verify the plumbing without waiting 12 hours. Watch:

```bash
sudo docker service logs -f postgres-backup_backup
# Expect:
#   [...Z] postgres-backup booting
#   [...Z]   host: postgres_primary:5432  db: kevsrobots_cms
#   [...Z]   storage: daily=/backups/daily (keep 28)
#   [...Z]            weekly=/backups/weekly (keep 4)
#   [...Z] backup start  -> /backups/daily/kevsrobots_cms_…dump
#   [...Z] backup ok     (12M)
#   [...Z] sleeping 21452s until next slot
```

Verify a dump landed on the NAS:

```bash
ls -lh <NAS mount on a workstation>/daily/
# Should show one ${db}_${ts}.dump file.
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
| Service stuck in `Preparing` forever               | NFS mount failing. Check `docker service ps postgres-backup_backup --no-trunc` for the error, then `showmount -e NAS_IP` from the host. |
| Logs show `pg_dump: connection to server failed`   | Wrong network. Re-check the `networks:` block — must match the postgres overlay (`docker network ls`).            |
| Logs show `pg_dump: error: server version mismatch`| Database is on a newer Postgres than `postgres:16-alpine`. Bump the image tag in compose to match.                |
| Daily folder filling up past 28 files              | The prune step needs write access to its own files on NFS — confirm `no_root_squash` (or matching uid/gid) on the NFS export. |
| No backups at all after deploy                     | NAS IP / export path placeholders still unedited.                                                                 |

## Why this design

* **Sleep loop, not cron.** Docker Swarm has no native cron and adding
  a cron daemon means a second supervisor process. A pure `sleep`
  loop respects SIGTERM (trap → graceful exit) and is the same
  total code regardless.
* **Same secrets as the postgres service.** Means no credential
  duplication; rotating either secret automatically rotates the
  backup credentials too.
* **NFS as a Docker volume.** Docker handles the mount lifecycle.
  If the NAS reboots, Docker reconnects when it's back without
  manual mount-fix work.
* **Custom format dumps.** Restore is one `pg_restore` command,
  supports parallel restore, and contents are inspectable via
  `pg_restore -l` without decompressing first.
