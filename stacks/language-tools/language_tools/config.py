"""Application configuration via pydantic-settings."""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path
from typing import Optional

from pydantic import computed_field
from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        extra="ignore",
    )

    port: int = 8310

    # The project editor on www.kevsrobots.com calls this with the session
    # cookie, so it needs CORS with credentials for the site's origin.
    cors_origins: str = "https://www.kevsrobots.com,http://localhost:4000"

    @computed_field
    @property
    def cors_origins_list(self) -> list[str]:
        return [s.strip() for s in self.cors_origins.split(",") if s.strip()]

    # Chatter JWT secret — must match chatter's SECRET_KEY (and the Projects
    # API's JWT_SECRET) to verify the session token.
    jwt_secret: str = "changeme"
    jwt_algorithm: str = "HS256"

    # A LanguageTool server for grammar, e.g. http://127.0.0.1:8010. Empty
    # turns grammar off (the editor keeps its own built-in rules).
    languagetool_url: str = ""

    # A LibreTranslate server for Translate. Off unless set.
    libretranslate_url: str = ""
    libretranslate_api_key: str = ""

    # WordNet's dict/ directory (index.noun, data.noun…) for Look Up and
    # Synonyms. The Docker image puts it at /usr/share/wordnet.
    wordnet_dir: Optional[Path] = None

    # Per signed-in user, per minute. Stops the service being a free
    # LanguageTool proxy, and keeps one tab from starving the Pi.
    check_per_minute: int = 30
    lookup_per_minute: int = 120
    translate_per_minute: int = 20


@lru_cache
def get_settings() -> Settings:
    return Settings()
