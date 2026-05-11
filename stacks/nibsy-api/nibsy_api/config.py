"""Application configuration via pydantic-settings.

Reads from environment variables and an optional `.env` file. Defaults are
suitable for local development against the docker-compose Postgres.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

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

    # Service port — #70 standardises on 8200.
    port: int = 8200


def get_settings() -> Settings:
    """Return a fresh Settings instance.

    Kept as a callable so tests can monkeypatch the environment before
    instantiating, and so FastAPI dependency overrides work cleanly.
    """

    return Settings()
