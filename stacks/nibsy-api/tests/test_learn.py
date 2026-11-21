"""Learning recommendations: pathways, pins and projects from the catalogue (#632)."""

from __future__ import annotations

import pytest
from sqlalchemy import select

from nibsy_api.ingest import ingest_catalogue
from nibsy_api.models import NibsyContent


def _course(slug: str, tags: list[str], **extra) -> dict:
    return {
        "type": "course",
        "title": f"Course {slug}",
        "url": f"/learn/{slug}/00_intro.html",
        "description": f"All about {' and '.join(tags)}",
        "tags": tags,
        "date": "2026-01-01",
        "cover": f"/learn/{slug}/cover.jpg",
        "level": "beginner",
        "duration": "0h 30m",
        **extra,
    }


CATALOGUE = {
    "items": [
        {
            "type": "post",
            "title": "Servo tester",
            "url": "/blog/servo-tester.html",
            "description": "A servo tester with a Pico and MicroPython",
            "tags": ["servo", "pico", "micropython"],
            "date": "2026-02-01",
        },
        {
            "type": "post",
            "title": "Pinned post",
            "url": "/blog/pinned.html",
            "description": "Gardening tips",
            "tags": ["garden"],
            "date": "2026-02-01",
            "pins": ["/learn/rust/00_intro.html"],
        },
        _course("basics", ["micropython", "pico"]),
        _course("servos", ["servo", "micropython", "pico"], pins=["/projects/arm/index.html"]),
        _course("rust", ["rust"]),
        {
            "type": "project",
            "title": "Robot arm",
            "url": "/projects/arm/index.html",
            "description": "A Pico robot arm",
            "tags": ["robot arm", "pico"],
        },
        {
            "type": "project",
            "title": "Servo dog",
            "url": "/projects/dog/index.html",
            "description": "A walking robot dog driven by servos and a Pico",
            "tags": ["servo", "pico", "micropython"],
        },
        # A showcase project that is really a blog post.
        {"type": "project", "title": "Servo tester", "url": "/blog/servo-tester.html", "tags": []},
    ],
    "pathways": [
        {
            "slug": "micropython",
            "title": "MicroPython on the Pico",
            "url": "/learn/learning_pathways/micropython.html",
            "courses": [
                {"url": "/learn/basics/00_intro.html", "stage": "start"},
                {"url": "/learn/servos/00_intro.html", "stage": "build"},
                {"url": "/learn/rust/00_intro.html", "stage": "further"},
            ],
        }
    ],
}


@pytest.mark.asyncio
async def test_courses_carry_their_pathways_and_projects_are_ingested(session) -> None:
    await ingest_catalogue(CATALOGUE, session)
    servos = await session.scalar(select(NibsyContent).where(NibsyContent.url == "/learn/servos/00_intro.html"))
    assert servos.content_metadata["pathways"] == [
        {
            "slug": "micropython",
            "title": "MicroPython on the Pico",
            "url": "/learn/learning_pathways/micropython.html",
            "stage": "build",
            "position": 1,
            "of": 3,
        }
    ]
    assert servos.content_metadata["pins"] == ["/projects/arm/index.html"]
    post = await session.scalar(select(NibsyContent).where(NibsyContent.url == "/blog/servo-tester.html"))
    assert post.content_type == "post"
    assert post.content_metadata.get("project") is True
    projects = (await session.scalars(select(NibsyContent).where(NibsyContent.content_type == "project"))).all()
    assert {p.url for p in projects} == {"/projects/arm/index.html", "/projects/dog/index.html"}


@pytest.mark.asyncio
async def test_next_course_follows_the_pathway(client, session) -> None:
    await ingest_catalogue(CATALOGUE, session)
    response = await client.get("/api/recommendations/next-course?current=/learn/basics/00_intro.html")
    body = response.json()
    assert body["source"] == "pathway"
    assert body["pathway"] == "MicroPython on the Pico"
    assert body["next_course"]["url"] == "/learn/servos/00_intro.html"


@pytest.mark.asyncio
async def test_for_page_returns_related_courses_only(client, session) -> None:
    await ingest_catalogue(CATALOGUE, session)
    response = await client.get("/api/learn/for-page?page=/blog/servo-tester.html&limit=2")
    body = response.json()
    urls = [c["url"] for c in body["courses"]]
    assert urls[0] == "/learn/servos/00_intro.html"
    assert all(c["type"] == "course" for c in body["courses"])
    assert body["courses"][0]["cover"] == "/learn/servos/cover.jpg"
    assert "/learn/rust/00_intro.html" not in urls


@pytest.mark.asyncio
async def test_for_page_puts_pins_first_even_when_unrelated(client, session) -> None:
    await ingest_catalogue(CATALOGUE, session)
    body = (await client.get("/api/learn/for-page?page=/blog/pinned.html")).json()
    assert body["courses"][0]["url"] == "/learn/rust/00_intro.html"
    assert body["courses"][0]["reason"] == "Recommended by Kev"


@pytest.mark.asyncio
async def test_for_page_unknown_page_is_empty(client, session) -> None:
    await ingest_catalogue(CATALOGUE, session)
    body = (await client.get("/api/learn/for-page?page=/nowhere.html")).json()
    assert body["courses"] == []


@pytest.mark.asyncio
async def test_follow_on_gives_next_course_and_projects(client, session) -> None:
    await ingest_catalogue(CATALOGUE, session)
    # Any lesson of the course resolves to the course.
    body = (await client.get("/api/learn/follow-on?course=/learn/servos/05_wiring.html&limit=3")).json()
    assert body["course"]["url"] == "/learn/servos/00_intro.html"
    assert [n["course"]["url"] for n in body["next"]] == ["/learn/rust/00_intro.html"]
    assert body["next"][0]["pathway"]["title"] == "MicroPython on the Pico"
    project_urls = [p["url"] for p in body["projects"]]
    # The pinned project leads, then related things to build.
    assert project_urls[0] == "/projects/arm/index.html"
    assert "/projects/dog/index.html" in project_urls
    assert "/blog/servo-tester.html" in project_urls
    assert all(not u.startswith("/learn/") for u in project_urls)


@pytest.mark.asyncio
async def test_follow_on_last_course_has_no_next(client, session) -> None:
    await ingest_catalogue(CATALOGUE, session)
    body = (await client.get("/api/learn/follow-on?course=/learn/rust/00_intro.html")).json()
    assert body["next"] == []
