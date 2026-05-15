"""Application configuration via pydantic-settings."""

from __future__ import annotations

import secrets
from pathlib import Path
from typing import Optional

from pydantic import computed_field, Field
from pydantic_settings import BaseSettings, SettingsConfigDict


# Stable per-process salt fallback so download dedup keeps working across
# repeated calls to ``get_settings()`` even when the env var isn't set.
# This is overridden by the IP_HASH_SALT env var when present.
_PROCESS_IP_HASH_SALT_FALLBACK = secrets.token_hex(32)


class Settings(BaseSettings):
    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        extra="ignore",
    )

    database_url: str = "sqlite+aiosqlite:///./projects.db"
    port: int = 8300

    cors_origins: str = "https://www.kevsrobots.com,http://localhost:4000"

    @computed_field
    @property
    def cors_origins_list(self) -> list[str]:
        return [s.strip() for s in self.cors_origins.split(",") if s.strip()]

    # Chatter JWT secret — must match chatter's SECRET_KEY to verify tokens.
    jwt_secret: str = "changeme"
    jwt_algorithm: str = "HS256"

    # Base URL for the Chatter auth service. Used by the parts catalog
    # account-age gate to look up `account_created_at` from `/api/me`.
    chatter_base_url: str = "https://chatter.kevsrobots.com"

    # NAS storage
    nas_host: str = "192.168.1.79"
    nas_username: Optional[str] = None
    nas_password: Optional[str] = None
    nas_share_name: str = "chatter"

    # Admin usernames (comma-separated)
    admin_usernames: str = "kev"

    @computed_field
    @property
    def admin_usernames_list(self) -> list[str]:
        return [s.strip() for s in self.admin_usernames.split(",") if s.strip()]

    # File limits
    max_file_size: int = 25 * 1024 * 1024  # 25MB
    max_image_size: int = 10 * 1024 * 1024  # 10MB
    max_project_storage: int = 50 * 1024 * 1024  # 50MB total per project

    # IP hashing for download dedup — never expose raw IPs in API responses.
    # In production, set this via env (e.g. IP_HASH_SALT=...) to a long random
    # value so hashes are not predictable. Default is a per-process random
    # value that's stable for the life of this Python process — fine for tests
    # and dev, but loses dedup across restarts unless IP_HASH_SALT is set.
    ip_hash_salt: str = Field(default_factory=lambda: _PROCESS_IP_HASH_SALT_FALLBACK)


def get_settings() -> Settings:
    return Settings()
