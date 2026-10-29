#!/usr/bin/env bash
# Vendor Mermaid Factory's mermaid-tools library into the site.
#
# The project editor uses it for ```mermaid blocks (the diagram, and the
# split code + diagram editor while you edit one), and projects/view.html to
# draw them. It's an ES module that loads each diagram type as a chunk on
# first use, so the whole esm/ folder is copied.
#
# Usage: scripts/update-mermaid-tools.sh [path/to/mermaid-factory]
#        (defaults to ../Mermaid-factory beside this repo)

set -euo pipefail

here="$(cd "$(dirname "$0")/.." && pwd)"
factory="${1:-$here/../Mermaid-factory}"
frontend="$factory/frontend"
dest="$here/web/assets/js/mermaid-tools"

if [ ! -f "$frontend/package.json" ]; then
  echo "No mermaid-factory checkout at $factory" >&2
  exit 1
fi

(cd "$frontend" && npm ci --no-audit --no-fund && npm run build)

rm -rf "$dest"
mkdir -p "$dest"
cp -r "$frontend/dist/esm/." "$dest/"
node --check "$dest/mermaid-tools.js" 2>/dev/null || node --input-type=module --check < "$dest/mermaid-tools.js"

version="$(sed -n 's/^version *= *"\(.*\)"/\1/p' "$factory/pyproject.toml")"
commit="$(git -C "$factory" rev-parse --short HEAD 2>/dev/null || echo unknown)"
echo "mermaid-tools $version (mermaid-factory $commit)" > "$dest/VERSION"
echo "Updated $dest to mermaid-tools $version ($commit)"
