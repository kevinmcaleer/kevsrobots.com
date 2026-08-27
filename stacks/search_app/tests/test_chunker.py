"""Chunking is the highest-leverage part of the pipeline, so it gets the most tests."""

import pytest

from rag.chunker import _clean_metadata, breadcrumb, chunk_page, split_long_text


class TestBreadcrumb:
    def test_joins_title_and_trail(self):
        assert breadcrumb("SMARS", "Wiring > Motors") == "SMARS > Wiring > Motors"

    def test_collapses_repeated_title(self):
        # Pages open with an H1 repeating the title - do not say it twice.
        assert breadcrumb("SMARS", "SMARS > Motors") == "SMARS > Motors"

    def test_collapse_is_case_insensitive(self):
        assert breadcrumb("SMARS", "smars > Motors") == "SMARS > Motors"

    def test_drops_empty_trail_entries(self):
        # A page jumping from h1 to h3 leaves a padding gap in the trail.
        assert breadcrumb("SMARS", "Wiring >  > Motors") == "SMARS > Wiring > Motors"

    def test_title_only(self):
        assert breadcrumb("SMARS", "") == "SMARS"


class TestSplitLongText:
    def test_short_text_is_one_piece(self):
        assert split_long_text("short", size=100) == ["short"]

    def test_splits_on_paragraphs(self):
        text = "\n\n".join(["word " * 40] * 6)
        pieces = split_long_text(text, size=400, overlap=50)
        assert len(pieces) > 1
        # Nothing should be wildly over the window.
        assert all(len(p) < 400 * 1.6 for p in pieces)

    def test_overlap_carries_context_forward(self):
        text = "A" * 300 + "\n\n" + "B" * 300 + "\n\n" + "C" * 300
        pieces = split_long_text(text, size=400, overlap=100)
        assert len(pieces) > 1
        # Each later piece should start with the tail of the previous one.
        assert any("A" in p and "B" in p for p in pieces)

    def test_hard_splits_an_unbreakable_paragraph(self):
        # A pasted log file with no blank lines must not become one huge chunk.
        pieces = split_long_text("x" * 5000, size=500, overlap=50)
        assert len(pieces) > 1
        assert all(len(p) <= 500 * 1.5 for p in pieces)

    def test_no_empty_pieces(self):
        pieces = split_long_text("\n\n".join(["para " * 50] * 5), size=300, overlap=40)
        assert all(p.strip() for p in pieces)


class TestCleanMetadata:
    def test_drops_none_values(self):
        # A None value nulls the ENTIRE metadata dict in Chroma, silently.
        clean = _clean_metadata({"a": "x", "b": None})
        assert "b" not in clean
        assert clean["a"] == "x"

    def test_empty_list_becomes_placeholder(self):
        assert _clean_metadata({"tags": []})["tags"] == ["none"]

    def test_empty_string_becomes_placeholder(self):
        assert _clean_metadata({"course": ""})["course"] == "none"

    def test_numbers_and_bools_survive(self):
        clean = _clean_metadata({"n": 3, "f": 1.5, "b": True})
        assert clean == {"n": 3, "f": 1.5, "b": True}

    def test_list_items_are_stringified(self):
        assert _clean_metadata({"tags": ["a", 2]})["tags"] == ["a", "2"]


class TestChunkPage:
    def test_produces_chunks_with_breadcrumbs(self, sample_page):
        chunks = chunk_page(sample_page)
        assert chunks
        for chunk in chunks:
            assert chunk.text.startswith("Building a SMARS Robot")

    def test_ids_are_deterministic(self, sample_page):
        first = [c.id for c in chunk_page(sample_page)]
        second = [c.id for c in chunk_page(sample_page)]
        assert first == second
        assert first[0] == "learn/smars/01_intro.html::0"

    def test_metadata_is_complete(self, sample_page):
        meta = chunk_page(sample_page)[0].metadata
        for key in ("url", "title", "section", "page_type", "date", "content_hash"):
            assert key in meta, f"missing {key}"
        assert meta["page_type"] == "lesson"

    def test_short_sections_are_folded_in_not_lost(self, sample_page):
        """A one-line section is real content and must survive chunking.

        "The left motor uses GP6 and GP7" is short enough to fall under
        MIN_CHUNK_SIZE, and is exactly the kind of fact people search for.
        It gets folded into the preceding chunk, keeping its heading as a
        label, rather than being dropped.
        """
        text = " ".join(c.text for c in chunk_page(sample_page))
        assert "GP6 and GP7" in text
        assert "Left motor" in text

    def test_all_section_content_survives(self, sample_page):
        text = " ".join(c.text for c in chunk_page(sample_page))
        for fact in ("DRV8833", "N20 motors", "modular robot"):
            assert fact in text, f"lost {fact!r} during chunking"
