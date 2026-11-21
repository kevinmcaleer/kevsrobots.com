#!/usr/bin/env bash
# Build the MicroPython unix port that scripts/check_course_code.py runs lesson
# code on. Takes a couple of minutes once; after that it's cached.
#
#   scripts/get_micropython.sh            # builds into ~/.cache/kevsrobots/micropython
#   MP_VERSION=v1.26.1 scripts/get_micropython.sh /some/dir
#
# Needs git, make and a C compiler (build-essential / Xcode command line tools).
set -euo pipefail
VERSION="${MP_VERSION:-v1.26.1}"
DEST="${1:-$HOME/.cache/kevsrobots/micropython}"
BIN="$DEST/micropython"
if [ -x "$BIN" ] && "$BIN" -c "import sys; assert '.'.join(map(str, sys.implementation.version[:3])) == '${VERSION#v}'" 2>/dev/null; then
  echo "$BIN"
  exit 0
fi
SRC="$(mktemp -d)"
trap 'rm -rf "$SRC"' EXIT
git clone --quiet --depth 1 --branch "$VERSION" https://github.com/micropython/micropython "$SRC/micropython" >&2
make -s -C "$SRC/micropython/mpy-cross" -j4 >&2
make -s -C "$SRC/micropython/ports/unix" submodules >&2
make -s -C "$SRC/micropython/ports/unix" -j4 >&2
mkdir -p "$DEST"
cp "$SRC/micropython/ports/unix/build-standard/micropython" "$BIN"
echo "$BIN"
