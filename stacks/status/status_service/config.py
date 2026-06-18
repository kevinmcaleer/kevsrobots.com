"""Application configuration for the status service.

Reads from environment variables and an optional ``.env`` file. All
defaults are safe for local development. The production deploy supplies
``STATUS_SERVICES`` and ``STATUS_TUNNEL_TOKEN`` via ``.env``.

The service list is configurable via env (no hard-coded URLs in code, per
issue #203). The format is a comma-separated list of ``name=url`` pairs.
"""

from __future__ import annotations

import os
from typing import List, Tuple

from pydantic import computed_field
from pydantic_settings import BaseSettings, SettingsConfigDict


# The nine services named in the issue (in canonical display order). These
# are the confirmed production URLs (kept in sync with ``.env.example``);
# override any of them in ``.env`` if a host changes. Notes on the
# non-obvious ones:
#   * ``stats`` answers on ``/api/health`` (not ``/health``).
#   * ``page_count`` is deployed under the *underscored* subdomain
#     ``page_count.kevsrobots.com`` — the hyphenated host 404s, which the
#     poller would report as amber/"degraded".
#   * ``status`` self-polls on loopback so the default works before the
#     public tunnel is up; ``.env`` swaps in the public hostname to
#     exercise the full Cloudflare chain.
#   * ``random_facts`` currently does NOT resolve (no DNS for any host
#     variant) — it will read red until the service is renamed or removed
#     from this list. Pending a decommission/rename decision.
DEFAULT_SERVICES = (
    "search=https://search.kevsrobots.com/health,"
    "stats=https://stats.kevsrobots.com/api/health,"
    "page_count=https://page_count.kevsrobots.com/health,"
    "random_facts=https://random-facts.kevsrobots.com/health,"
    "nibsy=https://nibsy.kevsrobots.com/health,"
    "courses=https://courses.kevsrobots.com/health,"
    "chatter=https://chatter.kevsrobots.com/health,"
    "projects=https://projects.kevsrobots.com/health,"
    "status=http://127.0.0.1:8210/health"
)


class Settings(BaseSettings):
    """Runtime configuration."""

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        extra="ignore",
    )

    # Path to the SQLite database file. Production mounts /data so the
    # DB survives container restarts. Tests override to a temp file.
    status_db_path: str = "/data/status.db"

    # Service list, CSV of ``name=url`` pairs.
    status_services: str = DEFAULT_SERVICES

    # Polling cadence (minutes). Issue says 15 minutes. Lower in tests.
    poll_interval_minutes: int = 15

    # Per-check timeout (seconds). 200 within this window = green.
    poll_timeout_seconds: float = 5.0

    # Latency threshold (seconds) above which a 200 is downgraded to amber.
    amber_latency_seconds: float = 2.0

    # Retention window for the daily vacuum job.
    retention_days: int = 30

    # Hour/minute (UTC) when the daily vacuum runs.
    vacuum_hour_utc: int = 3
    vacuum_minute_utc: int = 30

    # Service port. Issue #203 leaves the port unspecified; pick 8210 to
    # sit alongside nibsy (8200) without collision.
    port: int = 8210

    # Granularity of the dashboard timeline (cells per day). 96 = 15-min
    # slots, which matches the poll interval one-for-one.
    timeline_cells_per_day: int = 96

    @computed_field
    @property
    def services_list(self) -> List[Tuple[str, str]]:
        """Parse ``status_services`` into ``[(name, url), ...]``.

        Bad entries (missing ``=``, empty name/url) are silently dropped
        so a typo in env can't crash startup.
        """

        out: List[Tuple[str, str]] = []
        for chunk in self.status_services.split(","):
            chunk = chunk.strip()
            if not chunk or "=" not in chunk:
                continue
            name, _, url = chunk.partition("=")
            name = name.strip()
            url = url.strip()
            if not name or not url:
                continue
            out.append((name, url))
        return out

    @computed_field
    @property
    def service_names(self) -> List[str]:
        return [n for n, _ in self.services_list]


def get_settings() -> Settings:
    """Return a fresh Settings instance.

    Called inline (not cached) so tests can monkeypatch the environment
    between calls without restart.
    """

    return Settings()


def scheduler_disabled() -> bool:
    """True when APScheduler should NOT start (tests / explicit opt-out).

    Mirrors ``projects-api``'s ``background_jobs_disabled`` pattern:
    either ``STATUS_DISABLE_SCHEDULER=1`` is set explicitly (the
    conftest does this) or we detect pytest is running.
    """

    if os.environ.get("STATUS_DISABLE_SCHEDULER", "").strip() in {"1", "true", "TRUE"}:
        return True
    if os.environ.get("PYTEST_CURRENT_TEST"):
        return True
    return False
