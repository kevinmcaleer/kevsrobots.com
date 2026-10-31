#!/usr/bin/env bash
# Vendor markdown-studio's Layers graphic editor into the site.
#
# The project editor's "/Graphic" block (and "Edit graphic" / "Draw on it"
# on pictures) loads it the first time a graphic is opened, as an ES module
# from web/assets/js/layers/. It carries Fabric.js inside, so it isn't part
# of the Markdown editor bundle.
#
# Usage: scripts/update-layers.sh [path/to/markdown-studio]
#        (defaults to ../markdown-studio beside this repo)

set -euo pipefail

here="$(cd "$(dirname "$0")/.." && pwd)"
studio="${1:-$here/../markdown-studio}"
pkg="$studio/packages/layers"
dest="$here/web/assets/js/layers"

if [ ! -f "$pkg/package.json" ]; then
  echo "No Layers package at $pkg" >&2
  exit 1
fi

(cd "$studio" && npm ci --no-audit --no-fund && npm run build -w packages/layers)

rm -rf "$dest"
mkdir -p "$dest"
# The modules without their source maps (not vendored), so drop the comments pointing at them.
for f in "$pkg"/dist/*.js; do
  sed '$ { /^\/\/# sourceMappingURL=/d; }' "$f" > "$dest/$(basename "$f")"
  node --input-type=module --check < "$dest/$(basename "$f")"
done

version="$(node -p "require('$pkg/package.json').version")"
commit="$(git -C "$studio" rev-parse --short HEAD 2>/dev/null || echo unknown)"
echo "layers $version (markdown-studio $commit)" > "$dest/VERSION"
echo "Updated $dest to layers $version ($commit)"
