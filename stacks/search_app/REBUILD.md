# Rebuilding and deploying search

Search has two indexes, both built **into the Docker image** from the freshly
built Jekyll site:

| Index | What it is | Used by |
|---|---|---|
| `search.db` | SQLite FTS5, keyword/BM25 | `/search/`, autocomplete |
| `chroma/` | ChromaDB vectors, semantic | `/search/semantic`, `/mcp` |

Because both are baked in, each node serves from its own local copy. There is no
shared database, nothing to sync, and no runtime indexing step. Publishing
content and redeploying is all it takes for search to be current.

---

## Deploy order is not optional

The search page ships in the **site** image; the semantic API ships in the
**search** image. They deploy independently, so either can be newer than the
other. Both directions are now handled — the page falls back to keyword search
when the semantic endpoint is absent, and hides the MCP invite until the API
confirms it serves one — but deploy in the order below anyway, or users get
keyword-quality results until the search image lands.

**This bit once.** The site image went out with a page calling
`/search/semantic`, the search image did not follow, and the old API answered
404 with a JSON body. The fallback checked only for a structured "unavailable"
reply, so it never fired and the page rendered nothing at all, with no error.
`tests/frontend/test_fallback.js` now covers that and four related failure modes.

## Deploy: the whole flow

The site image is the source of the content, so it must be built **first**.

```bash
# 1. On the build Pi - build and push the site image
cd ~/ClusteredPi/stacks/kevsrobots
docker build -t 192.168.2.1:5000/kevsrobots:latest .
docker push 192.168.2.1:5000/kevsrobots:latest

# 2. Build and push the search image (indexes are built during this step)
cd ~/kevsrobots.com/stacks/search_app
docker build -t 192.168.2.1:5000/search:latest .
docker push 192.168.2.1:5000/search:latest

# 3. On the node running search
cd ~/ClusteredPi/stacks/search
docker compose -f docker-compose.prod.yml pull
docker compose -f docker-compose.prod.yml up -d

# 4. Verify
curl -s https://search.kevsrobots.com/health | jq
```

A healthy response looks like:

```json
{
  "status": "healthy",
  "fts_documents": 1240,
  "vector_chunks": 9376,
  "mcp": "mounted",
  "problems": []
}
```

**If you build the search image on a machine that did not just build the site
image**, pull it first, or you will index a stale site:

```bash
docker pull 192.168.2.1:5000/kevsrobots:latest
```

To index a site image other than `latest`:

```bash
docker build --build-arg SITE_IMAGE=192.168.2.1:5000/kevsrobots:some-tag -t ... .
```

---

## What the image build does

1. Pulls the built `_site` out of the site image (`/www/data`)
2. Warms the embedding model into the image, so no container ever downloads
   ~80MB of ONNX model on first request
3. Builds the FTS index from the site HTML
4. Builds the vector index: chunk on headings, add breadcrumbs, embed
5. Runs `build_index.py --stats`, which **fails the build if either index is
   empty** — better a failed build than an image that answers every query with
   "no results" while looking perfectly healthy

Expect **5-15 minutes on a Pi 5**, almost all of it embedding ~9,400 chunks.

The ARM64 layers are verified: all wheels install from prebuilt binaries (no
source builds, no Rust toolchain needed), and the ONNX embedding model runs on
aarch64. Throughput was only measured under QEMU emulation, which is several
times slower than native — hence the wide range.

---

## Local development

```bash
cd stacks/search_app
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt

# Build the Jekyll site first, then:
.venv/bin/python build_index.py           # both indexes
.venv/bin/python build_index.py --stats   # what is in them
.venv/bin/python build_index.py --summary # what the chunker produces, no indexing

.venv/bin/uvicorn app:app --reload
```

Re-running `build_index.py` is incremental: only pages whose content changed are
re-embedded, so a second run takes seconds rather than minutes.

Change chunking, the embedding model, or the metadata schema? Use `--full`.
Incremental indexing skips unchanged pages, so they keep their **old** chunk
boundaries and the index becomes a mixture of two strategies — worse than
either alone.

---

## Testing

```bash
./run_tests.sh
```

Runs the unit and API tests, then the retrieval evaluation, which fails if the
hit rate drops below 85%. Worth running before any deploy that touches
chunking or retrieval — a bad chunking tweak can leave every unit test green
while search quality collapses.

Current baseline (23 golden questions, k=10):

| Engine | Hit rate | MRR |
|---|---|---|
| Hybrid (semantic + exact) | 100% | 0.721 |
| FTS5 keyword only | 61% | 0.502 |

Add questions to `eval/eval.yml` whenever you notice search getting something
wrong — the miss list is the to-do list.

---

## Troubleshooting

**`curl /health` returns 503** — read `problems` in the response. An empty index
means the build did not see the site; check the site image was pulled.

**Semantic search returns nothing, keyword works** — the vector index is missing
or the RAG import failed. `docker logs <container>` will show the import error,
and `/health` reports `mcp: unavailable`.

**Results got worse after a change** — rebuild with `--full`. This is the first
thing to try, every time.

**Build fails at the `FROM ${SITE_IMAGE}` line** — the registry is unreachable
or the tag does not exist. `docker pull 192.168.2.1:5000/kevsrobots:latest`.

**MCP returns 421 / `Invalid Host header`** — the `Host` header is not in
`MCP_ALLOWED_HOSTS`. The defaults cover `search.kevsrobots.com`, `localhost` and
`127.0.0.1` on any port. Serving on a new hostname needs that env var set
(comma-separated); entries without a port automatically match any port.

Note that `curl` will not reproduce this if you pass `-H 'Host: ...'` yourself —
test with a real client, or omit the header so curl sends the host you dialled.
