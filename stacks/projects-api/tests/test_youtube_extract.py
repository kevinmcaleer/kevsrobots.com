"""Unit tests for the YouTube id extractor (issue #171).

Hits the helper directly so we don't need a client fixture — the
parser is plain Python with no DB / network dependency.
"""

from __future__ import annotations

import pytest

from projects_api.youtube import extract_youtube_id


def test_bare_id_round_trips() -> None:
    """An 11-char id is returned as-is, preserving case."""
    assert extract_youtube_id("dQw4w9WgXcQ") == "dQw4w9WgXcQ"


def test_bare_id_mixed_case_preserved() -> None:
    """Case is part of the id — never lowercased."""
    assert extract_youtube_id("AbCdEfGhIjK") == "AbCdEfGhIjK"


def test_watch_url() -> None:
    assert (
        extract_youtube_id("https://www.youtube.com/watch?v=dQw4w9WgXcQ")
        == "dQw4w9WgXcQ"
    )


def test_watch_url_with_extra_params() -> None:
    """Extra query params (t, list, feature) shouldn't trip the parser."""
    url = "https://www.youtube.com/watch?v=dQw4w9WgXcQ&t=42s&list=PL12345"
    assert extract_youtube_id(url) == "dQw4w9WgXcQ"


def test_watch_url_without_www() -> None:
    assert (
        extract_youtube_id("https://youtube.com/watch?v=dQw4w9WgXcQ")
        == "dQw4w9WgXcQ"
    )


def test_short_url() -> None:
    assert extract_youtube_id("https://youtu.be/dQw4w9WgXcQ") == "dQw4w9WgXcQ"


def test_short_url_with_query() -> None:
    assert (
        extract_youtube_id("https://youtu.be/dQw4w9WgXcQ?t=10")
        == "dQw4w9WgXcQ"
    )


def test_embed_url() -> None:
    assert (
        extract_youtube_id("https://www.youtube.com/embed/dQw4w9WgXcQ")
        == "dQw4w9WgXcQ"
    )


def test_shorts_url() -> None:
    assert (
        extract_youtube_id("https://www.youtube.com/shorts/dQw4w9WgXcQ")
        == "dQw4w9WgXcQ"
    )


def test_m_subdomain() -> None:
    """Mobile subdomain (``m.youtube.com``) is treated like the main site."""
    assert (
        extract_youtube_id("https://m.youtube.com/watch?v=dQw4w9WgXcQ")
        == "dQw4w9WgXcQ"
    )


def test_legacy_v_path() -> None:
    """Old share URL shape ``/v/<id>`` — cheap to keep working."""
    assert (
        extract_youtube_id("https://www.youtube.com/v/dQw4w9WgXcQ")
        == "dQw4w9WgXcQ"
    )


@pytest.mark.parametrize(
    "bad",
    [
        "",
        "   ",
        "not-a-url",
        "https://example.com/watch?v=dQw4w9WgXcQ",  # wrong host
        "https://vimeo.com/123456",
        "https://www.youtube.com/watch",  # missing v=
        "https://www.youtube.com/watch?v=tooshort",
        "https://www.youtube.com/watch?v=way_too_long_for_an_id",
        "https://youtu.be/",  # empty path
        "https://www.youtube.com/embed/",  # empty embed path
        "dQw4w9WgXc",  # 10 chars
        "dQw4w9WgXcQQ",  # 12 chars
        "dQw4w9WgX!Q",  # bad char
    ],
)
def test_malformed_raises(bad: str) -> None:
    with pytest.raises(ValueError):
        extract_youtube_id(bad)


def test_none_raises() -> None:
    with pytest.raises(ValueError):
        extract_youtube_id(None)  # type: ignore[arg-type]
