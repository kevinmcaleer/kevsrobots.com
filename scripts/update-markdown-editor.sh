#!/usr/bin/env bash
# Vendor markdown-studio's standalone Markdown editor into the site.
#
# The project editor (web/projects/editor.html) loads it as a plain <script>
# from web/assets/js/markdown-editor/. This rebuilds it from a markdown-studio
# checkout and copies the bundle, its stylesheet and the spelling word lists.
#
# Usage: scripts/update-markdown-editor.sh [path/to/markdown-studio]
#        (defaults to ../markdown-studio beside this repo)

set -euo pipefail

here="$(cd "$(dirname "$0")/.." && pwd)"
studio="${1:-$here/../markdown-studio}"
pkg="$studio/packages/markdown-editor"
dest="$here/web/assets/js/markdown-editor"

if [ ! -f "$pkg/package.json" ]; then
  echo "No markdown-studio checkout at $studio" >&2
  exit 1
fi

(cd "$studio" && npm ci --no-audit --no-fund && npm run build -w packages/markdown-editor)

mkdir -p "$dest/dictionaries"
# The source map isn't vendored, so drop the comment pointing at it (only
# the last line: the inlined spelling worker carries one of its own).
sed '$ { /^\/\/# sourceMappingURL=/d; }' "$pkg/dist/markdown-editor.js" > "$dest/markdown-editor.js"
node --check "$dest/markdown-editor.js"
cp "$pkg/dist/markdown-editor.css" "$dest/markdown-editor.css"
cp "$pkg/dist/dictionaries/"* "$dest/dictionaries/"

version="$(node -p "require('$pkg/package.json').version")"
commit="$(git -C "$studio" rev-parse --short HEAD 2>/dev/null || echo unknown)"
echo "markdown-editor $version (markdown-studio $commit)" > "$dest/VERSION"
echo "Updated $dest to markdown-editor $version ($commit)"
