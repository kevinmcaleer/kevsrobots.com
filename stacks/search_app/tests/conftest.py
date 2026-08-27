"""Shared fixtures.

The chunker and content tests run against synthetic HTML so they are fast and
deterministic. The retrieval tests need the real index, and skip themselves if
it has not been built.
"""

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))


@pytest.fixture
def sample_html():
    return """
    <html>
      <head>
        <title>Building a SMARS Robot</title>
        <meta property="description" content="Build a small modular robot from 3D printed parts">
        <meta property="page-type" content="lesson">
        <meta property="date" content="2026-01-15T00:00:00+00:00">
        <meta property="author" content="Kevin McAleer">
        <meta property="og:image" content="http://0.0.0.0:4000/assets/cover.jpg">
      </head>
      <body>
        <nav><a href="/">Home</a><a href="/blog">Blog</a></nav>
        <div class="learn-nav">Lesson 1 Lesson 2 40% Percent Complete</div>
        <div class="col-lg-9">
          <h1>Building a SMARS Robot</h1>
          <h2>Build a small modular robot from 3D printed parts</h2>
          <p>SMARS is a modular robot you can print at home.</p>
          <h2>Motor wiring</h2>
          <p>The DRV8833 takes four GPIO pins, two per motor.</p>
          <h3>Left motor</h3>
          <p>The left motor uses GP6 and GP7.</p>
          <h2>Parts list</h2>
          <p>You need two N20 motors and a Raspberry Pi Pico.</p>
        </div>
        <footer>Copyright Kevin McAleer</footer>
      </body>
    </html>
    """


@pytest.fixture
def sample_page(tmp_path, sample_html, monkeypatch):
    from rag import config

    site = tmp_path / "site"
    site.mkdir()
    page_file = site / "learn" / "smars" / "01_intro.html"
    page_file.parent.mkdir(parents=True)
    page_file.write_text(sample_html, encoding="utf-8")

    monkeypatch.setattr(config, "SITE_ROOT", site)
    from rag import content

    monkeypatch.setattr(content, "SITE_ROOT", site)
    content._site_metadata.cache_clear()
    content.course_lookup.cache_clear()
    content.page_metadata_lookup.cache_clear()

    return content.parse_page(page_file)


@pytest.fixture(scope="session")
def live_index():
    """The real vector index, or skip if it has not been built."""
    try:
        from rag.vector_index import collection_stats

        info = collection_stats()
    except Exception as error:
        pytest.skip(f"vector index unavailable: {error}")
    if not info.get("chunks"):
        pytest.skip("vector index is empty - run build_index.py first")
    return info
