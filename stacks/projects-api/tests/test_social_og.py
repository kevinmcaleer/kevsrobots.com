"""Tests for the social-card OG endpoint (GET /og/<owner>/<slug>)."""

from __future__ import annotations

import pytest

from .conftest import make_auth_header


async def _create_project(
    client,
    *,
    title: str,
    username: str = "testuser",
    short_description: str | None = None,
    tags: list[str] | None = None,
    cover_image: str | None = None,
) -> dict:
    """POST a project and (optionally) PUT cover_image after-the-fact.

    ``cover_image`` is not part of the ``ProjectCreate`` schema so it can
    only be set via PUT — we do that here when the test asks for one.
    """
    payload: dict = {"title": title}
    if short_description is not None:
        payload["short_description"] = short_description
    if tags is not None:
        payload["tags"] = tags
    headers = make_auth_header(username)
    response = await client.post("/api/projects", json=payload, headers=headers)
    assert response.status_code == 201, response.text
    body = response.json()

    if cover_image is not None:
        put = await client.put(
            f"/api/projects/{body['id']}",
            json={"cover_image": cover_image},
            headers=headers,
        )
        assert put.status_code == 200, put.text
        body = put.json()

    return body


# ---- Happy path --------------------------------------------------------


@pytest.mark.asyncio
async def test_found_project_returns_html_with_og_title(client) -> None:
    project = await _create_project(
        client,
        title="My Burgerbot",
        short_description="A robot that flips burgers.",
    )
    response = await client.get(
        f"/og/testuser/{project['slug']}"
    )
    assert response.status_code == 200
    assert response.headers["content-type"].startswith("text/html")
    body = response.text
    assert 'property="og:title"' in body
    assert 'content="My Burgerbot"' in body
    # Twitter card present.
    assert 'name="twitter:card"' in body
    assert "summary_large_image" in body
    # Canonical URL points at the public site.
    assert (
        'href="https://www.kevsrobots.com/projects/testuser/'
        f'{project["slug"]}"'
    ) in body
    # Cache header is set so Cloudflare can cache the edge response.
    assert "max-age=300" in response.headers.get("cache-control", "")


# ---- Cover image absolutisation ---------------------------------------


@pytest.mark.asyncio
async def test_relative_cover_image_is_absolutized(client) -> None:
    project = await _create_project(
        client,
        title="Relative Cover",
        cover_image="/uploads/foo.jpg",
    )
    response = await client.get(f"/og/testuser/{project['slug']}")
    assert response.status_code == 200
    assert (
        'content="https://projects.kevsrobots.com/uploads/foo.jpg"'
        in response.text
    )


@pytest.mark.asyncio
async def test_absolute_cover_image_is_left_alone(client) -> None:
    project = await _create_project(
        client,
        title="Absolute Cover",
        cover_image="https://cdn.example.com/banner.png",
    )
    response = await client.get(f"/og/testuser/{project['slug']}")
    assert response.status_code == 200
    assert 'content="https://cdn.example.com/banner.png"' in response.text
    # Must not have been prefixed with the upload host.
    assert (
        "projects.kevsrobots.com/https://cdn.example.com" not in response.text
    )


@pytest.mark.asyncio
async def test_missing_cover_image_falls_back_to_default(client) -> None:
    project = await _create_project(client, title="No Cover")
    response = await client.get(f"/og/testuser/{project['slug']}")
    assert response.status_code == 200
    assert (
        'content="https://www.kevsrobots.com/assets/img/projects/'
        'projects-hub-og.jpg"'
    ) in response.text


# ---- Tags -------------------------------------------------------------


@pytest.mark.asyncio
async def test_tags_render_as_meta_tags(client) -> None:
    project = await _create_project(
        client,
        title="Tagged Project",
        tags=["robotics", "micropython", "3dprinting"],
    )
    response = await client.get(f"/og/testuser/{project['slug']}")
    assert response.status_code == 200
    body = response.text
    assert body.count('property="article:tag"') == 3
    assert 'content="robotics"' in body
    assert 'content="micropython"' in body
    assert 'content="3dprinting"' in body


# ---- Not found --------------------------------------------------------


@pytest.mark.asyncio
async def test_unknown_owner_slug_returns_404_generic_card(client) -> None:
    response = await client.get("/og/nobody/does-not-exist")
    assert response.status_code == 404
    # Still HTML, not a JSON detail body.
    assert response.headers["content-type"].startswith("text/html")
    body = response.text
    assert "<html" in body.lower()
    assert "KevsRobots Projects Hub" in body
    # Generic hub URL is canonical on the 404 card.
    assert "https://www.kevsrobots.com/projects/hub" in body
    # No FastAPI JSON detail leaked through.
    assert '"detail"' not in body
    # Negative response is also cached.
    assert "max-age=300" in response.headers.get("cache-control", "")


# ---- HTML escaping ----------------------------------------------------


@pytest.mark.asyncio
async def test_html_escaping_blocks_script_injection(client) -> None:
    # Stuff a script tag and a stray quote into the title + description.
    project = await _create_project(
        client,
        title='<script>alert(1)</script>"hello',
        short_description='quote " and <b>bold</b>',
    )
    response = await client.get(f"/og/testuser/{project['slug']}")
    assert response.status_code == 200
    body = response.text
    # No raw script tag in the output.
    assert "<script>alert(1)</script>" not in body
    # The dangerous chars are escaped as entities.
    assert "&lt;script&gt;" in body
    # The unescaped raw double-quote must not appear inside an attribute
    # value (would break out of the meta content=""). We assert the
    # escaped form is present.
    assert "&quot;hello" in body or "&#x27;hello" in body or '\\"hello' not in body
    # And the description's < / " are escaped too.
    assert "&lt;b&gt;bold&lt;/b&gt;" in body
