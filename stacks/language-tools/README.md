# Language tools

Grammar, Look Up (dictionary), Synonyms (thesaurus) and "Rewrite as
active" for the project editor (issue #256). Spelling needs none of this: the editor checks it in
the browser with its own Hunspell word lists, so it works offline.

| Endpoint | What it does |
| --- | --- |
| `GET /api/language` | Which features are on: `{lookup, grammar, translate, rewrite}` |
| `GET /api/language/define?word=…` | WordNet senses with examples and synonyms; 404 if unknown |
| `POST /api/language/check` | Grammar, proxied to LanguageTool (its spelling and style rules are skipped) |
| `GET /api/language/translate/languages`, `POST /api/language/translate` | LibreTranslate; off unless `LIBRETRANSLATE_URL` is set |
| `POST /api/language/rewrite` | `{text, phrase}`: one passive sentence rewritten in the active voice by Claude; off unless `ANTHROPIC_API_KEY` is set |
| `GET /health` | Public health check |

These are the endpoints markdown-studio's editor already calls
(`packages/markdown-editor/src/proofing/languageApi.ts`), so the editor needs
no changes to use them. Every `/api/language` endpoint needs a signed-in
user, from the `access_token` cookie (Chatter sets it for `.kevsrobots.com`)
or an `Authorization: Bearer` dev token, and is rate limited per user (30
grammar checks, 120 look-ups, 20 translations and 10 rewrites a minute by
default). If the
service is down, the editor quietly drops back to spelling and its built-in
grammar rules.

## Settings (`.env`)

| Variable | Default | |
| --- | --- | --- |
| `JWT_SECRET` | `changeme` | **Required.** Same as the Projects API's / Chatter's `SECRET_KEY` |
| `LANGUAGETOOL_URL` | empty (grammar off) | Set to `http://127.0.0.1:8010` by the compose file |
| `WORDNET_DIR` | `/usr/share/wordnet` in the image | WordNet's `dict/` files |
| `LIBRETRANSLATE_URL`, `LIBRETRANSLATE_API_KEY` | empty (off) | Optional translation |
| `CORS_ORIGINS` | `https://www.kevsrobots.com,https://kevsrobots.com,http://localhost:4000,http://local.kevsrobots.com:4000` | Origins allowed to call with credentials |
| `ANTHROPIC_API_KEY` | empty (rewrite off) | A Claude API key from console.anthropic.com, for "Rewrite as active" on the passive-voice hint. Mermaid Factory's AI box can use the same key |
| `REWRITE_MODEL` | `claude-opus-5-5` | The Claude model that rewrites (refusals fall back to another model automatically) |
| `CHECK_PER_MINUTE`, `LOOKUP_PER_MINUTE`, `TRANSLATE_PER_MINUTE`, `REWRITE_PER_MINUTE` | 30, 120, 20, 10 | Per-user limits; 0 turns a limit off |
| `LANGUAGE_TUNNEL_TOKEN` | | The Cloudflare tunnel for `language.kevsrobots.com` |

## Deploy

1. Nothing to build by hand: `docker compose up -d --build` (step 4) builds
   the image. `docker compose push language-tools` publishes it to the registry.
2. In Cloudflare Zero Trust, create a tunnel (or add a public hostname to the
   Projects API's tunnel) for `language.kevsrobots.com` → `http://localhost:8310`.
   If you reuse the Projects API's tunnel, drop the `cloudflared` service here.
3. `cp .env.example .env` and fill in `JWT_SECRET` and `LANGUAGE_TUNNEL_TOKEN`.
4. `docker compose up -d --build`. LanguageTool takes
   a minute or so to start, and uses about 1 GB of memory.
5. Check it: `curl http://127.0.0.1:8310/health` on the server, then
   `curl -i -H 'Origin: http://local.kevsrobots.com:4000' https://language.kevsrobots.com/api/language`
   from anywhere (a 401 with `access-control-allow-origin` is right when
   signed out; a Cloudflare 530/1033 means the tunnel isn't connected).

The editor's base URL is `LANGUAGE_API` in `web/assets/js/project-editor.js`.

## Tests

```bash
pip install -r requirements-dev.txt
python -m pytest
```
