#!/bin/bash
# Run the search test suite and the retrieval evaluation.
#
# Both indexes must exist - run build_index.py first. The retrieval tests skip
# themselves if the vector index is missing, so a bare unit-test run still works.
set -e
cd "$(dirname "$0")"

PY="${PYTHON:-python3}"
[ -x .venv/bin/python ] && PY=.venv/bin/python

echo "=== Unit and API tests ==="
$PY -m pytest tests/ -q

echo ""
echo "=== Frontend fallback tests ==="
# The search page and the search API ship in separate images and deploy
# independently, so the page must survive talking to an older API. This is
# what broke in production: a 404 from a not-yet-deployed endpoint rendered
# a blank page with no error.
if command -v node >/dev/null 2>&1; then
    node tests/frontend/test_fallback.js
else
    echo "SKIPPED - node not installed"
fi

echo ""
echo "=== Retrieval evaluation ==="
# --fail-under guards against a change that quietly breaks retrieval: a bad
# chunking tweak can leave every test green while search gets much worse.
$PY eval/evaluate.py --fail-under 0.85
