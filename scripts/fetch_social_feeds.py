#!/usr/bin/env python3
"""Refresh web/_data/social_feed.yml with Kev's latest Instagram, TikTok and X posts.

The bio page (web/about/bio.md -> _includes/social_feed.html) renders these
URLs with each platform's official embed. YouTube is not handled here: it
already comes from web/_data/youtube.yml.

Each platform is optional and only runs when its credentials are set. If a
platform has no credentials, or its API call fails, the posts already in the
data file are kept, so hand-pasted posts are never wiped.

Environment variables
---------------------
Instagram (Instagram API with Instagram Login; needs a Creator/Business account)
  INSTAGRAM_ACCESS_TOKEN   long-lived token (valid 60 days; this script refreshes
                           it, and prints the new one so you can update the secret)

TikTok (Display API, scope video.list)
  TIKTOK_CLIENT_KEY
  TIKTOK_CLIENT_SECRET
  TIKTOK_REFRESH_TOKEN     from the one-off OAuth login (valid 365 days)

X (API v2)
  X_BEARER_TOKEN           app bearer token
  X_USER_ID                optional; numeric id of @kevsmac, saves one API call

Usage
-----
  python3 scripts/fetch_social_feeds.py            # update the data file
  python3 scripts/fetch_social_feeds.py --dry-run  # print what would be written
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import urllib.error
import urllib.parse
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

try:
    import yaml
except ImportError:  # pragma: no cover
    sys.exit("PyYAML is required: pip install pyyaml")

DATA_FILE = Path(__file__).resolve().parent.parent / "web" / "_data" / "social_feed.yml"
MAX_POSTS = 6
TIMEOUT = 20

HEADER = """\
# Social feed shown on /about/bio (see _includes/social_feed.html)
#
# YouTube comes from _data/youtube.yml, so it needs nothing here.
#
# Instagram, TikTok and X:
#   - Paste post URLs into `posts` by hand (newest first) and they render
#     with each platform's official embed. No API keys needed.
#   - Or set API tokens and run `python3 scripts/fetch_social_feeds.py`
#     (the "Refresh social feeds" GitHub Action does this on a schedule).
#     The script only replaces a platform's posts when its fetch succeeds,
#     so hand-curated posts are never wiped by a failed or missing token.
#
# With no posts listed, the bio page falls back to:
#   Instagram -> a "follow" card, TikTok -> TikTok's creator embed (shows the
#   latest videos automatically), X -> X's profile timeline embed.

"""


def http_json(url: str, *, method: str = "GET", headers: dict | None = None,
              data: dict | None = None) -> dict:
    body = None
    headers = dict(headers or {})
    headers.setdefault("User-Agent", "kevsrobots-social-feed/1.0")
    if data is not None:
        body = urllib.parse.urlencode(data).encode()
        headers.setdefault("Content-Type", "application/x-www-form-urlencoded")
    req = urllib.request.Request(url, data=body, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=TIMEOUT) as resp:
        return json.load(resp)


# --- Instagram -----------------------------------------------------------------

def fetch_instagram() -> list[dict] | None:
    token = os.environ.get("INSTAGRAM_ACCESS_TOKEN")
    if not token:
        return None

    # Long-lived tokens last 60 days; refreshing extends them. Only possible
    # once the token is at least 24h old, so a failure here is not fatal.
    try:
        refreshed = http_json(
            "https://graph.instagram.com/refresh_access_token?"
            + urllib.parse.urlencode({"grant_type": "ig_refresh_token", "access_token": token})
        )
        if refreshed.get("access_token") and refreshed["access_token"] != token:
            token = refreshed["access_token"]
            print("instagram: token refreshed; update the INSTAGRAM_ACCESS_TOKEN secret", file=sys.stderr)
    except urllib.error.URLError as exc:
        print(f"instagram: token refresh skipped ({exc})", file=sys.stderr)

    resp = http_json(
        "https://graph.instagram.com/me/media?"
        + urllib.parse.urlencode({
            "fields": "permalink,timestamp,media_type",
            "limit": MAX_POSTS,
            "access_token": token,
        })
    )
    return [
        {"url": item["permalink"], "date": item.get("timestamp", "")[:10]}
        for item in resp.get("data", [])
        if item.get("permalink")
    ][:MAX_POSTS]


# --- TikTok --------------------------------------------------------------------

def fetch_tiktok(handle: str) -> list[dict] | None:
    key = os.environ.get("TIKTOK_CLIENT_KEY")
    secret = os.environ.get("TIKTOK_CLIENT_SECRET")
    refresh = os.environ.get("TIKTOK_REFRESH_TOKEN")
    if not (key and secret and refresh):
        return None

    # Access tokens only last 24h, so mint a fresh one from the refresh token.
    tok = http_json(
        "https://open.tiktokapis.com/v2/oauth/token/",
        method="POST",
        data={
            "client_key": key,
            "client_secret": secret,
            "grant_type": "refresh_token",
            "refresh_token": refresh,
        },
    )
    access = tok.get("access_token")
    if not access:
        raise RuntimeError(f"no access_token in TikTok response: {tok.get('error_description') or tok}")
    if tok.get("refresh_token") and tok["refresh_token"] != refresh:
        print("tiktok: refresh token rotated; update the TIKTOK_REFRESH_TOKEN secret", file=sys.stderr)

    req = urllib.request.Request(
        "https://open.tiktokapis.com/v2/video/list/?fields=id,share_url,create_time",
        data=json.dumps({"max_count": MAX_POSTS}).encode(),
        headers={
            "Authorization": f"Bearer {access}",
            "Content-Type": "application/json",
            "User-Agent": "kevsrobots-social-feed/1.0",
        },
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=TIMEOUT) as resp:
        videos = json.load(resp).get("data", {}).get("videos", [])

    posts = []
    for v in videos[:MAX_POSTS]:
        url = (v.get("share_url") or f"https://www.tiktok.com/@{handle}/video/{v['id']}").split("?")[0]
        date = ""
        if v.get("create_time"):
            date = datetime.fromtimestamp(int(v["create_time"]), tz=timezone.utc).strftime("%Y-%m-%d")
        posts.append({"url": url, "date": date})
    return posts


# --- X -------------------------------------------------------------------------

def fetch_x(handle: str) -> list[dict] | None:
    bearer = os.environ.get("X_BEARER_TOKEN")
    if not bearer:
        return None
    headers = {"Authorization": f"Bearer {bearer}"}

    user_id = os.environ.get("X_USER_ID")
    if not user_id:
        user_id = http_json(f"https://api.x.com/2/users/by/username/{handle}", headers=headers)["data"]["id"]

    resp = http_json(
        f"https://api.x.com/2/users/{user_id}/tweets?"
        + urllib.parse.urlencode({
            "max_results": 5,  # API minimum; keeps reads low on the free tier
            "exclude": "replies,retweets",
            "tweet.fields": "created_at",
        }),
        headers=headers,
    )
    return [
        {"url": f"https://x.com/{handle}/status/{t['id']}", "date": t.get("created_at", "")[:10]}
        for t in resp.get("data", [])
    ][:MAX_POSTS]


# --- Main ----------------------------------------------------------------------

def dump(data: dict) -> str:
    return HEADER + yaml.safe_dump(data, sort_keys=False, allow_unicode=True, width=1000)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    parser.add_argument("--dry-run", action="store_true", help="print the result instead of writing it")
    args = parser.parse_args()

    data = yaml.safe_load(DATA_FILE.read_text()) or {}
    changed = False

    fetchers = {
        "instagram": lambda: fetch_instagram(),
        "tiktok": lambda: fetch_tiktok(data["tiktok"]["handle"]),
        "x": lambda: fetch_x(data["x"]["handle"]),
    }
    for name, fetch in fetchers.items():
        try:
            posts = fetch()
        except Exception as exc:  # keep going; one broken platform shouldn't block the rest
            print(f"{name}: fetch failed, keeping existing posts ({exc})", file=sys.stderr)
            continue
        if posts is None:
            print(f"{name}: no credentials set, keeping existing posts", file=sys.stderr)
            continue
        if not posts:
            print(f"{name}: API returned no posts, keeping existing posts", file=sys.stderr)
            continue
        if posts != data[name].get("posts"):
            data[name]["posts"] = posts
            changed = True
        print(f"{name}: {len(posts)} posts", file=sys.stderr)

    if changed:
        data["updated"] = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")

    out = dump(data)
    if args.dry_run:
        print(out)
    elif changed:
        DATA_FILE.write_text(out)
        print(f"wrote {DATA_FILE}", file=sys.stderr)
    else:
        print("no changes", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
