"""Application configuration via pydantic-settings.

Reads from environment variables and an optional `.env` file. Defaults are
suitable for local development against the docker-compose Postgres.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

from pydantic import computed_field
from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    """Runtime configuration."""

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        extra="ignore",
    )

    # SQLAlchemy async URL. Falls back to local aiosqlite for cheap dev/test.
    # Production (see #70) will override to asyncpg against 192.168.2.1:5433.
    database_url: str = "sqlite+aiosqlite:///./nibsy.db"

    # Path to a directory containing the YAML data files. If set and the
    # `nibsy_content` table is empty on startup, ingestion runs automatically.
    nibsy_data_dir: Optional[Path] = None

    # Comma-separated allowed origins for CORS. Stored as a plain string
    # so pydantic-settings doesn't try to JSON-parse it.
    cors_origins: str = "https://www.kevsrobots.com,http://localhost:4000"

    @computed_field
    @property
    def cors_origins_list(self) -> list[str]:
        return [s.strip() for s in self.cors_origins.split(",") if s.strip()]

    # Base URL of the live site for remote ingestion (#69).
    site_base_url: str = "https://www.kevsrobots.com"

    # Pagecount database URL for trending scores (#72).
    # Direct DB query instead of HTTP to avoid inflating visit counts.
    page_count_url: str = ""

    # Hour (UTC) at which the daily remote ingest runs. Set negative to disable.
    ingest_schedule_hour: int = 1

    # Service port — #70 standardises on 8200.
    port: int = 8200

    # How often the recommendations generator (#74) re-runs in the background.
    # Production default is 14 days; tests/dev can shorten it via env var.
    # Set to 0 or negative to disable the scheduler entirely.
    recommendation_refresh_days: int = 14

    # ---- Auth / admin (#158) --------------------------------------------
    # Chatter signs every access_token with this secret. Must match the
    # value in chatter's SECRET_KEY (and the JWT_SECRET in projects-api)
    # so all three services trust the same tokens.
    jwt_secret: str = "changeme"
    jwt_algorithm: str = "HS256"

    # Base URL for the Chatter auth service. Reserved for endpoints that
    # need to read the full /api/me payload (avatar, account_created_at,
    # etc.). The default admin gating only needs to verify the JWT.
    chatter_base_url: str = "https://chatter.kevsrobots.com"

    # Comma-separated allow-list of usernames that may hit /api/admin/*.
    admin_usernames: str = "kev"

    @computed_field
    @property
    def admin_usernames_list(self) -> list[str]:
        return [s.strip() for s in self.admin_usernames.split(",") if s.strip()]


def get_settings() -> Settings:
    """Return a fresh Settings instance.

    Kept as a callable so tests can monkeypatch the environment before
    instantiating, and so FastAPI dependency overrides work cleanly.
    """

    return Settings()
