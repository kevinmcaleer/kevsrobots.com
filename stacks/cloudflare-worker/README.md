# Cloudflare Worker — Projects Hub social cards

## Purpose

Project view pages on `www.kevsrobots.com` live at
`/projects/<owner>/<slug>` and are fully client-rendered: a static
`view.html` boots, fetches project JSON from
`https://projects.kevsrobots.com`, and assembles the DOM in the
browser. Social-card crawlers (Twitterbot, facebookexternalhit,
Slackbot, Discordbot, LinkedInBot, WhatsApp, TelegramBot,
Mastodon-pulled fetchers, Iframely, Embedly, etc.) do not execute
JavaScript, so links to project pages currently unfurl as "Page Not
Found" — they only see the SPA shell's static `<head>`. This Worker
sits at the edge in front of the site, sniffs the `User-Agent`, and
when the requester is a known crawler hitting a two-segment project
URL it swaps the response for a tiny OG-only HTML document fetched
from the projects-api `GET /og/<owner>/<slug>` endpoint. Humans never
see the swap — they get the normal SPA.

## Why this isn't just an nginx rewrite

Cloudflare sits in front of nginx (CDN, SSL, DDoS, caching). Doing the
UA-sniff at the Worker layer means:

- The rewrite happens **at the edge** — no extra origin hop for any
  crawler request, and the cached OG response can be served from any
  Cloudflare PoP worldwide.
- **No nginx config change** has to be rolled out across the four
  backend Pis (and the load-balancer Pi) every time the crawler list
  or routing logic changes. Worker deploys are a single `wrangler
  publish` (or one click in the dashboard).
- The Worker fails open: on any error fetching the API it falls
  through to origin, so the worst case is the same broken preview the
  site has today — never a 5xx from us.

## Dependency

This Worker is useless without the matching projects-api endpoint:

- `GET https://projects.kevsrobots.com/og/<owner>/<slug>` —
  returns `text/html` with OG / Twitter card meta tags. Defined in
  `stacks/projects-api/projects_api/routers/social_og.py`. **Deploy
  the API first**, then this Worker.

## Deployment

Two equivalent options — pick whichever workflow you prefer.

### Option A — Wrangler CLI

Standard Cloudflare Worker deploy. See the official docs:
<https://developers.cloudflare.com/workers/wrangler/>.

The short version is `wrangler init`, drop `social-og.js` in as the
entry-point, set the route to `www.kevsrobots.com/projects/*` (in
`wrangler.toml`), then `wrangler publish`.

### Option B — Dashboard

Cloudflare Workers can also be created and edited entirely from the
dashboard: <https://developers.cloudflare.com/workers/get-started/dashboard/>.

Paste the contents of `social-og.js` into a new Worker, add a route
for `www.kevsrobots.com/projects/*` on the kevsrobots.com zone, save
and deploy.

## Local testing

```bash
wrangler dev social-og.js
# In another terminal:
curl -i \
  -A 'Twitterbot/1.0' \
  http://127.0.0.1:8787/projects/kev/burgerbot

# Should return text/html with og:title set, X-OG-Worker: hit header.

# Sanity check: a human UA falls through (in dev this is a 404 because
# wrangler dev doesn't proxy origin, but you should NOT see the
# X-OG-Worker header).
curl -i \
  -A 'Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7) Chrome/120.0' \
  http://127.0.0.1:8787/projects/kev/burgerbot
```

## Post-deploy verification

After publishing, hit each of the four standard social-card validators
with a real project URL:

- Twitter / X — <https://cards-dev.twitter.com/validator>
- Facebook — <https://developers.facebook.com/tools/debug/>
- LinkedIn — <https://www.linkedin.com/post-inspector/>
- Generic — <https://opengraph.xyz/>

Plus a one-liner smoke test from the command line:

```bash
curl -A 'Twitterbot/1.0' https://www.kevsrobots.com/projects/kev/burgerbot
```

You should see an HTML document with `og:title`, `og:description`,
`og:image`, `twitter:card`, and the canonical URL. The same URL hit
without the crawler UA should return the regular SPA HTML (the
JS-rendered project page).

## Notes

- The crawler list is conservative — better to over-detect (a real
  user with a quirky UA gets a redirect to the same page) than
  under-detect and miss platforms whose previews are then broken.
- The Worker caches each OG response for 5 minutes at the edge
  (`cf: { cacheTtl: 300, cacheEverything: true }`) so a project that
  gets shared 50 times in five minutes generates one API hit, not 50.
- The `X-OG-Worker: hit` response header is purely for debugging —
  it's how you confirm the swap is happening in production without
  having to UA-spoof from your laptop every time.
