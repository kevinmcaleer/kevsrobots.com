from __future__ import annotations

from datetime import datetime
from pathlib import Path
from urllib.parse import urlsplit

from fastapi import FastAPI, Query, Request
from fastapi.responses import PlainTextResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates

_HERE = Path(__file__).resolve().parent
_TEMPLATES = Jinja2Templates(directory=str(_HERE / "templates"))


def _valid_timestamp(value: str) -> bool:
    if not value:
        return False
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return False
    return parsed.tzinfo is not None


def _valid_image_url(value: str) -> bool:
    if not value:
        return True
    parsed = urlsplit(value)
    return parsed.scheme in {"http", "https"} and bool(parsed.netloc)


def create_app() -> FastAPI:
    app = FastAPI(
        title="KevsRobots Countdown",
        description="Create and share a countdown to any date.",
        version="1.0.0",
    )
    app.mount(
        "/static",
        StaticFiles(directory=str(_HERE / "static")),
        name="static",
    )

    @app.middleware("http")
    async def security_headers(request: Request, call_next):
        response = await call_next(request)
        response.headers["Content-Security-Policy"] = (
            "default-src 'self'; "
            "img-src 'self' http: https:; "
            "script-src 'self'; "
            "style-src 'self'; "
            "frame-ancestors *; "
            "base-uri 'none'; "
            "form-action 'self'"
        )
        response.headers["Referrer-Policy"] = "strict-origin-when-cross-origin"
        response.headers["X-Content-Type-Options"] = "nosniff"
        return response

    @app.get("/health", response_class=PlainTextResponse)
    async def health() -> str:
        return "ok"

    @app.get("/")
    async def countdown(
        request: Request,
        at: str = Query(default="", max_length=40),
        description: str = Query(default="", max_length=280),
        image: str = Query(default="", max_length=2048),
    ):
        error = ""
        if at and not _valid_timestamp(at):
            error = "The shared countdown date is invalid. Please create a new one."
            at = ""
        if image and not _valid_image_url(image):
            error = "The image must use a valid http:// or https:// URL."
            image = ""

        response = _TEMPLATES.TemplateResponse(
            request,
            "index.html",
            {
                "at": at,
                "description": description,
                "image": image,
                "error": error,
            },
        )
        response.headers["Cache-Control"] = "no-store, max-age=0"
        return response

    return app


app = create_app()
