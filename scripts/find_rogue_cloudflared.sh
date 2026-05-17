#!/usr/bin/env bash
# Find (and optionally kill) a rogue cloudflared container running on
# the kevsrobots Pi cluster — one whose hostname defaulted to a Docker
# container short ID instead of the customised dev0X hostname.
#
# Usage:
#   scripts/find_rogue_cloudflared.sh                    # find + report
#   scripts/find_rogue_cloudflared.sh --kill             # find + stop & rm
#   scripts/find_rogue_cloudflared.sh --kill <id-prefix> # only kill the
#                                                       # matching container
#
# All four Pis share the same username and password (or SSH key). Set
# REMOTE_USER below or via env. If a password is required, install
# sshpass (brew install hudochenkov/sshpass/sshpass) and set
# REMOTE_PASS — but key auth is strongly preferred.

set -u

# Hosts to sweep. Override via env: NODES='dev01 raspberrypi.lan snaszy01.lan'
NODES=(${NODES:-dev01 dev02 dev03 dev04})
REMOTE_USER="${REMOTE_USER:-kev}"
TARGET_PREFIX="${TARGET_PREFIX:-64955431940d}"  # the rogue's short ID

KILL=0
KILL_PREFIX=""
for arg in "$@"; do
  case "$arg" in
    --kill) KILL=1 ;;
    --help|-h)
      sed -n '2,15p' "$0"; exit 0 ;;
    *) KILL=1; KILL_PREFIX="$arg" ;;
  esac
done

SSH_OPTS=(
  -o ConnectTimeout=5
  -o StrictHostKeyChecking=accept-new
  -o BatchMode=no   # let ssh prompt for password if no key available
)

ssh_run() {
  local host="$1"; shift
  if [[ -n "${REMOTE_PASS:-}" ]]; then
    if ! command -v sshpass >/dev/null 2>&1; then
      echo "REMOTE_PASS is set but sshpass is not installed. Install with:" >&2
      echo "  brew install hudochenkov/sshpass/sshpass" >&2
      exit 1
    fi
    sshpass -p "$REMOTE_PASS" ssh "${SSH_OPTS[@]}" "$REMOTE_USER@$host" "$@"
  else
    ssh "${SSH_OPTS[@]}" "$REMOTE_USER@$host" "$@"
  fi
}

found_host=""
found_id=""
found_image=""

for node in "${NODES[@]}"; do
  echo "=== $node ==="
  # List every container and its image; we grep for the rogue id prefix or
  # any container whose image is cloudflared (so we can spot other strays).
  out="$(ssh_run "$node" "docker ps --format '{{.ID}}|{{.Names}}|{{.Image}}|{{.RunningFor}}' 2>/dev/null | grep -E '${TARGET_PREFIX}|cloudflared' || true")"
  if [[ -z "$out" ]]; then
    echo "  (no cloudflared / rogue containers running)"
    continue
  fi
  echo "$out" | while IFS='|' read -r cid cname cimage cage; do
    echo "  id=$cid name=$cname image=$cimage age=$cage"
  done
  if echo "$out" | grep -q "^${TARGET_PREFIX}"; then
    found_host="$node"
    found_id="$(echo "$out" | grep "^${TARGET_PREFIX}" | head -1 | cut -d'|' -f1)"
    found_image="$(echo "$out" | grep "^${TARGET_PREFIX}" | head -1 | cut -d'|' -f3)"
  fi
done

echo
if [[ -z "$found_host" ]]; then
  echo "Rogue container with prefix '${TARGET_PREFIX}' was NOT found on any node."
  echo "If you saw a different short ID in the Cloudflare dashboard, re-run with:"
  echo "  TARGET_PREFIX=<new-id> $0"
  exit 1
fi

echo "Found rogue on $found_host: $found_id ($found_image)"
echo

echo "--- docker inspect (key metadata) ---"
ssh_run "$found_host" "docker inspect $found_id --format 'Name:           {{.Name}}
Image:          {{.Config.Image}}
Started:        {{.State.StartedAt}}
Net mode:       {{.HostConfig.NetworkMode}}
Compose project: {{ index .Config.Labels \"com.docker.compose.project\" }}
Compose service: {{ index .Config.Labels \"com.docker.compose.service\" }}
Command:        {{.Path}} {{.Args}}'"
echo

if [[ "$KILL" -eq 1 ]]; then
  if [[ -n "$KILL_PREFIX" && "$KILL_PREFIX" != "$found_id"* && "$found_id" != "$KILL_PREFIX"* ]]; then
    echo "Kill prefix '$KILL_PREFIX' does not match the discovered id '$found_id'. Aborting."
    exit 2
  fi
  echo "Killing $found_id on $found_host ..."
  ssh_run "$found_host" "docker stop $found_id && docker rm $found_id"
  echo "Done. Check the Cloudflare Tunnel connectors page — the entry should disappear within a minute."
else
  echo "Re-run with --kill to stop and remove it:"
  echo "  $0 --kill"
fi
