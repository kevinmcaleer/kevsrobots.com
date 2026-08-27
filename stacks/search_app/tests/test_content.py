"""Content extraction: the right text in, the boilerplate out, metadata attached."""

import pytest

from rag.content import _clean_image_url, _is_subtitle, _normalise_link


class TestBoilerplateStripping:
    def test_navigation_is_removed(self, sample_page):
        assert "Blog" not in sample_page.body_text or "Home" not in sample_page.body_text

    def test_course_sidebar_is_removed(self, sample_page):
        # .learn-nav is the lesson progress sidebar - it appears on every
        # lesson and would make them all look alike to the embedding model.
        assert "Percent Complete" not in sample_page.body_text

    def test_footer_is_removed(self, sample_page):
        assert "Copyright Kevin McAleer" not in sample_page.body_text

    def test_real_content_survives(self, sample_page):
        assert "DRV8833" in sample_page.body_text


class TestMetadata:
    def test_reads_meta_tags(self, sample_page):
        assert sample_page.title == "Building a SMARS Robot"
        assert sample_page.page_type == "lesson"
        assert sample_page.author == "Kevin McAleer"
        assert sample_page.description.startswith("Build a small modular robot")

    def test_url_is_site_relative(self, sample_page):
        assert sample_page.url == "learn/smars/01_intro.html"

    def test_dev_host_is_rewritten_in_cover(self, sample_page):
        assert sample_page.cover_image.startswith("https://www.kevsrobots.com")
        assert "0.0.0.0" not in sample_page.cover_image


class TestContentHash:
    def test_is_stable_across_calls(self, sample_page):
        assert sample_page.content_hash == sample_page.content_hash

    def test_changes_with_content(self, sample_page):
        original = sample_page.content_hash
        sample_page.sections[0].text += " Now with an extra sentence."
        assert sample_page.content_hash != original

    def test_is_short_enough_for_metadata(self, sample_page):
        assert len(sample_page.content_hash) == 16


class TestExcludedTypes:
    def test_utility_pages_are_skipped(self, tmp_path, monkeypatch):
        from rag import content

        site = tmp_path / "site"
        site.mkdir()
        page = site / "tags.html"
        page.write_text(
            '<html><head><title>Tags</title>'
            '<meta property="page-type" content="tag"></head>'
            '<body><p>every tag on the site</p></body></html>',
            encoding="utf-8",
        )
        monkeypatch.setattr(content, "SITE_ROOT", site)
        assert content.parse_page(page) is None


class TestHelpers:
    @pytest.mark.parametrize("raw,expected", [
        ("/learn/docker/", "learn/docker/index.html"),
        ("learn/docker/01.html", "learn/docker/01.html"),
        ("/blog/post.html#section", "blog/post.html"),
        ("", ""),
    ])
    def test_normalise_link(self, raw, expected):
        assert _normalise_link(raw) == expected

    def test_subtitle_detection_matches_description(self):
        assert _is_subtitle("Build a small robot", "Build a small robot")

    def test_subtitle_detection_allows_real_headings(self):
        assert not _is_subtitle("Motor wiring", "Build a small modular robot")

    def test_very_long_heading_is_a_subtitle(self):
        assert _is_subtitle("x" * 120, "")

    def test_relative_image_becomes_absolute(self):
        assert _clean_image_url("/a/b.jpg") == "https://www.kevsrobots.com/a/b.jpg"

    def test_http_image_becomes_https(self):
        assert _clean_image_url("http://www.kevsrobots.com/a.jpg").startswith("https://")
