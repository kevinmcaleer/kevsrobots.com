"""Retrieval: filters, ranking helpers, and the real index end to end."""

import pytest

from rag.retrieve import (
    Hit,
    build_filter,
    diversify,
    drop_stragglers,
    looks_like_identifier,
    merge_hits,
)


def _hit(url, distance=0.5, exact=False, chunk_index=0):
    return Hit(
        text=f"crumb\n\nbody of {url}",
        url=url,
        title=url,
        section="s",
        distance=distance,
        metadata={"chunk_index": chunk_index},
        exact=exact,
    )


class TestBuildFilter:
    def test_no_options_returns_none(self):
        # Chroma wants None for "no filter" - {} is an error.
        assert build_filter() is None

    def test_single_clause_is_not_wrapped_in_and(self):
        # $and requires at least two operands.
        assert build_filter(tag="pico") == {"tags": {"$contains": "pico"}}

    def test_two_clauses_are_combined(self):
        where = build_filter(tag="pico", course="micropython")
        assert "$and" in where and len(where["$and"]) == 2

    def test_tag_is_lowercased(self):
        # Tags are lowercased at index time, so queries must be too.
        assert build_filter(tag="MicroPython") == {"tags": {"$contains": "micropython"}}

    def test_single_page_type_uses_eq(self):
        assert build_filter(page_types=["lesson"]) == {"page_type": {"$eq": "lesson"}}

    def test_many_page_types_use_in(self):
        where = build_filter(page_types=["lesson", "blog"])
        assert where == {"page_type": {"$in": ["lesson", "blog"]}}

    def test_tags_use_contains_not_in(self):
        # $in compares the whole list against the string and matches nothing,
        # with no error - the classic Chroma trap.
        where = build_filter(tag="pico")
        assert "$contains" in where["tags"]


class TestDiversify:
    def test_caps_chunks_per_page(self):
        hits = [_hit("a.html", chunk_index=i) for i in range(5)]
        assert len(diversify(hits, limit=10, per_page=2)) == 2

    def test_lets_other_pages_through(self):
        hits = [_hit("a.html", chunk_index=0), _hit("a.html", chunk_index=1),
                _hit("a.html", chunk_index=2), _hit("b.html")]
        urls = [h.url for h in diversify(hits, limit=10, per_page=2)]
        assert urls == ["a.html", "a.html", "b.html"]

    def test_respects_the_limit(self):
        hits = [_hit(f"{i}.html") for i in range(20)]
        assert len(diversify(hits, limit=5)) == 5


class TestDropStragglers:
    def test_keeps_relatively_good_hits(self):
        hits = [_hit("a", distance=0.3), _hit("b", distance=0.5)]
        assert len(drop_stragglers(hits, ratio=0.5)) == 2

    def test_drops_far_worse_hits(self):
        hits = [_hit("a", distance=0.1), _hit("b", distance=0.95)]
        assert len(drop_stragglers(hits, ratio=0.5)) == 1

    def test_never_returns_empty_when_given_hits(self):
        hits = [_hit("a", distance=0.99)]
        assert drop_stragglers(hits, ratio=0.9) == hits

    def test_handles_empty_input(self):
        assert drop_stragglers([], ratio=0.5) == []

    def test_ratio_none_disables_it(self):
        hits = [_hit("a", distance=0.1), _hit("b", distance=0.99)]
        assert len(drop_stragglers(hits, ratio=None)) == 2


class TestIdentifiers:
    @pytest.mark.parametrize("term", ["PCA9685", "DRV8833", "TB6612FNG", "ESP32", "L298N"])
    def test_recognises_part_numbers(self, term):
        assert looks_like_identifier(term)

    @pytest.mark.parametrize("term", ["motor", "robot", "the", "pi", "a1"])
    def test_ignores_ordinary_words(self, term):
        assert not looks_like_identifier(term)


class TestMergeHits:
    def test_exact_matches_come_first(self):
        merged = merge_hits([_hit("x.html", exact=True)], [_hit("y.html")], limit=10)
        assert merged[0].url == "x.html"

    def test_deduplicates_the_same_chunk(self):
        same = _hit("x.html", chunk_index=3)
        merged = merge_hits([same], [_hit("x.html", chunk_index=3)], limit=10)
        assert len(merged) == 1

    def test_keeps_different_chunks_of_one_page(self):
        merged = merge_hits(
            [_hit("x.html", chunk_index=0)], [_hit("x.html", chunk_index=1)], limit=10
        )
        assert len(merged) == 2

    def test_respects_the_limit(self):
        merged = merge_hits([], [_hit(f"{i}.html") for i in range(20)], limit=3)
        assert len(merged) == 3


class TestHitScore:
    def test_distance_becomes_similarity(self):
        assert _hit("a", distance=0.25).score == pytest.approx(0.75)

    def test_score_is_clamped_to_zero(self):
        assert _hit("a", distance=1.8).score == 0.0

    def test_score_is_clamped_to_one(self):
        assert _hit("a", distance=-0.2).score == 1.0

    def test_as_result_has_the_api_shape(self):
        result = _hit("a.html").as_result()
        for key in ("url", "page_title", "snippet", "score", "page_type"):
            assert key in result

    def test_snippet_drops_the_breadcrumb(self):
        assert not _hit("a.html").as_result()["snippet"].startswith("crumb")


@pytest.mark.usefixtures("live_index")
class TestAgainstRealIndex:
    def test_semantic_finds_without_shared_keywords(self):
        from rag.retrieve import search

        hits = search("robot that draws pictures on a wall", top_k=5)
        assert hits, "no results at all"
        assert any("scrawly" in h.url.lower() for h in hits), [h.url for h in hits]

    def test_exact_identifier_wins(self):
        from rag.retrieve import search

        hits = search("PCA9685", top_k=5)
        assert hits
        assert hits[0].exact, "an exact identifier match should rank first"

    def test_page_type_filter_is_respected(self):
        from rag.retrieve import search

        hits = search("motors", top_k=8, page_types=["lesson"])
        assert hits
        assert all(h.metadata.get("page_type") == "lesson" for h in hits)

    def test_tag_filter_is_respected(self):
        from rag.retrieve import search

        hits = search("getting started", top_k=8, tag="micropython")
        assert hits
        assert all("micropython" in h.metadata.get("tags", []) for h in hits)

    def test_empty_query_returns_nothing(self):
        from rag.retrieve import search

        assert search("") == []
        assert search("   ") == []

    def test_results_are_page_diverse(self):
        from rag.retrieve import search

        hits = search("micropython", top_k=10)
        urls = [h.url for h in hits]
        assert len(set(urls)) >= len(urls) / 2, f"too repetitive: {urls}"

    def test_get_page_chunks_returns_document_order(self):
        from rag.retrieve import get_page_chunks, search

        hits = search("micropython", top_k=1)
        chunks = get_page_chunks(hits[0].url)
        assert chunks
        indices = [c["metadata"].get("chunk_index", 0) for c in chunks]
        assert indices == sorted(indices)
