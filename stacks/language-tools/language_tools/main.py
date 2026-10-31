"""FastAPI application entrypoint for the language tools service."""

from __future__ import annotations

import logging

import httpx
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from .config import get_settings
from .language import Language, make_router

logging.basicConfig(level=logging.INFO)


def create_app(transport: httpx.AsyncBaseTransport | None = None) -> FastAPI:
    settings = get_settings()
    language = Language(settings, transport)

    app = FastAPI(title="kevsrobots language tools", docs_url=None, redoc_url=None)
    app.add_middleware(
        CORSMiddleware,
        allow_origins=settings.cors_origins_list,
        allow_credentials=True,
        allow_methods=["GET", "POST"],
        allow_headers=["Authorization", "Content-Type", "Accept"],
    )

    @app.get("/health")
    def health() -> dict:
        return {"status": "ok", **language.features()}

    app.include_router(make_router(language))
    return app


app = create_app()
