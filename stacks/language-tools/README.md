# Language tools

Grammar, Look Up (dictionary) and Synonyms (thesaurus) for the project
editor (issue #256). Spelling needs none of this: the editor checks it in
the browser with its own Hunspell word lists, so it works offline.

| Endpoint | What it does |
| --- | --- |
| `GET /api/language` | Which features are on: `{lookup, grammar, translate}` |
| `GET /api/language/define?word=…` | WordNet senses with examples and synonyms; 404 if unknown |
| `POST /api/language/check` | Grammar, proxied to LanguageTool (its spelling and style rules are skipped) |
| `GET /api/language/translate/languages`, `POST /api/language/translate` | LibreTranslate; off unless `LIBRETRANSLATE_URL` is set |
| `GET /health` | Public health check |

These are the endpoints markdown-studio's editor already calls
(`packages/markdown-editor/src/proofing/languageApi.ts`), so the editor needs
no changes to use them. Every `/api/language` endpoint needs a signed-in
user, from the `access_token` cookie (Chatter sets it for `.kevsrobots.com`)
or an `Authorization: Bearer` dev token, and is rate limited per user (30
grammar checks, 120 look-ups and 20 translations a minute by default). If the
service is down, the editor quietly drops back to spelling and its built-in
grammar rules.

## Settings (`.env`)

| Variable | Default | |
| --- | --- | --- |
| `JWT_SECRET` | `changeme` | **Required.** Same as the Projects API's / Chatter's `SECRET_KEY` |
| `LANGUAGETOOL_URL` | empty (grammar off) | Set to `http://127.0.0.1:8010` by the compose file |
| `WORDNET_DIR` | `/usr/share/wordnet` in the image | WordNet's `dict/` files |
| `LIBRETRANSLATE_URL`, `LIBRETRANSLATE_API_KEY` | empty (off) | Optional translation |
| `CORS_ORIGINS` | `https://www.kevsrobots.com,http://localhost:4000` | Origins allowed to call with credentials |
| `CHECK_PER_MINUTE`, `LOOKUP_PER_MINUTE`, `TRANSLATE_PER_MINUTE` | 30, 120, 20 | Per-user limits; 0 turns a limit off |
| `LANGUAGE_TUNNEL_TOKEN` | | The Cloudflare tunnel for `language.kevsrobots.com` |

## Deploy

1. Build and push the image:
   `docker build -t 192.168.2.1:5000/language-tools:latest . && docker push 192.168.2.1:5000/language-tools:latest`
2. In Cloudflare Zero Trust, create a tunnel (or add a public hostname to the
   Projects API's tunnel) for `language.kevsrobots.com` → `http://localhost:8310`.
   If you reuse the Projects API's tunnel, drop the `cloudflared` service here.
3. `docker compose -f docker-compose.prod.yml up -d`. LanguageTool takes
   a minute or so to start, and uses about 1 GB of memory.

The editor's base URL is `LANGUAGE_API` in `web/assets/js/project-editor.js`.

## Tests

```bash
pip install -r requirements-dev.txt
python -m pytest
```
