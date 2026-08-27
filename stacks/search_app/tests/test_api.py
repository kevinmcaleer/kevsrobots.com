"""API surface: both search engines, the page reader, health, and MCP."""

import warnings

import pytest

warnings.filterwarnings("ignore")


@pytest.fixture(scope="module")
def client():
    from fastapi.testclient import TestClient

    import app as appmod

    with TestClient(appmod.app) as test_client:
        yield test_client


class TestHealth:
    def test_reports_index_sizes(self, client):
        body = client.get("/health").json()
        assert "fts_documents" in body
        assert "vector_chunks" in body

    def test_healthy_when_indexes_are_populated(self, client):
        response = client.get("/health")
        body = response.json()
        if body["fts_documents"] and body["vector_chunks"]:
            assert response.status_code == 200
            assert body["status"] == "healthy"
        else:
            # An empty index must read as an outage, not a healthy service.
            assert response.status_code == 503

    def test_empty_index_is_unhealthy(self, client, monkeypatch):
        import app as appmod

        monkeypatch.setattr(appmod, "fts_document_count", lambda: 0)
        response = client.get("/health")
        assert response.status_code == 503
        assert "FTS index is empty" in response.json()["problems"]


class TestSemanticSearch:
    def test_returns_results(self, client):
        body = client.get("/search/semantic", params={"query": "micropython"}).json()
        assert body["results"]
        assert body["engine"] in ("hybrid", "semantic")

    def test_result_shape_matches_the_legacy_api(self, client):
        body = client.get("/search/semantic", params={"query": "robot"}).json()
        row = body["results"][0]
        for key in ("url", "cover_image", "page_title", "description",
                    "date", "author", "page_type", "snippet"):
            assert key in row, f"missing {key}"

    def test_semantic_finds_by_meaning(self, client):
        body = client.get(
            "/search/semantic", params={"query": "robot that draws on a wall"}
        ).json()
        assert any("scrawly" in r["url"].lower() for r in body["results"])

    def test_page_type_filter(self, client):
        body = client.get(
            "/search/semantic", params={"query": "motors", "page_types": "lesson"}
        ).json()
        assert body["results"]
        assert all(r["page_type"] == "lesson" for r in body["results"])

    def test_pagination_returns_different_rows(self, client):
        params = {"query": "raspberry pi", "page_size": 3}
        first = client.get("/search/semantic", params={**params, "page": 1}).json()
        second = client.get("/search/semantic", params={**params, "page": 2}).json()
        assert [r["url"] for r in first["results"]] != [r["url"] for r in second["results"]]

    def test_total_count_describes_the_whole_set(self, client):
        # Not just the requested slice, or pagination and facets would lie.
        body = client.get(
            "/search/semantic", params={"query": "pico", "page_size": 3}
        ).json()
        assert body["total_count"] > 3

    def test_facets_are_returned(self, client):
        body = client.get("/search/semantic", params={"query": "robot"}).json()
        assert body["facets"]

    def test_empty_query_is_handled(self, client):
        body = client.get("/search/semantic", params={"query": ""}).json()
        assert body["results"] == []

    def test_nonsense_query_does_not_error(self, client):
        response = client.get("/search/semantic", params={"query": "qqzzxx not a word"})
        assert response.status_code == 200


class TestLegacySearch:
    def test_still_works(self, client):
        body = client.get("/search/", params={"query": "micropython"}).json()
        assert body["total_count"] > 0

    def test_shape_is_unchanged(self, client):
        body = client.get("/search/", params={"query": "robot"}).json()
        for key in ("results", "total_count", "total_pages", "page", "page_size", "facets"):
            assert key in body

    def test_fts_special_characters_are_sanitized(self, client):
        # An unbalanced quote is a syntax error in FTS5 MATCH.
        response = client.get("/search/", params={"query": 'robot" OR *'})
        assert response.status_code == 200


class TestPageEndpoint:
    def test_returns_chunks_for_a_real_page(self, client):
        hit = client.get("/search/semantic", params={"query": "micropython"}).json()
        url = hit["results"][0]["url"]
        body = client.get("/page/", params={"url": url}).json()
        assert body["chunk_count"] > 0
        assert body["chunks"]

    def test_unknown_page_is_empty_not_an_error(self, client):
        body = client.get("/page/", params={"url": "nope/does-not-exist.html"}).json()
        assert body["chunk_count"] == 0


class TestMCP:
    HEADERS = {
        "Accept": "application/json, text/event-stream",
        "Content-Type": "application/json",
    }

    def _rpc(self, client, method, params=None, request_id=1):
        payload = {"jsonrpc": "2.0", "id": request_id, "method": method}
        if params is not None:
            payload["params"] = params
        return client.post("/mcp/", json=payload, headers=self.HEADERS)

    def test_is_mounted(self, client):
        import app as appmod

        assert appmod.MCP_AVAILABLE, appmod.MCP_IMPORT_ERROR

    def test_initialize_handshake(self, client):
        response = self._rpc(client, "initialize", {
            "protocolVersion": "2025-06-18",
            "capabilities": {},
            "clientInfo": {"name": "pytest", "version": "1"},
        })
        assert response.status_code == 200
        assert response.json()["result"]["serverInfo"]["name"] == "kevsrobots"

    def test_lists_both_tools(self, client):
        body = self._rpc(client, "tools/list", request_id=2).json()
        names = {t["name"] for t in body["result"]["tools"]}
        assert names == {"search_kevsrobots", "get_kevsrobots_page"}

    def test_tools_describe_themselves(self, client):
        body = self._rpc(client, "tools/list", request_id=3).json()
        for tool in body["result"]["tools"]:
            assert len(tool.get("description", "")) > 80, f"{tool['name']} needs a real description"

    def test_search_tool_returns_citable_urls(self, client):
        body = self._rpc(client, "tools/call", {
            "name": "search_kevsrobots",
            "arguments": {"query": "how do I wire a servo to a pico", "limit": 3},
        }, request_id=4).json()
        text = body["result"]["content"][0]["text"]
        assert "https://www.kevsrobots.com/" in text

    def test_search_tool_honours_filters(self, client):
        body = self._rpc(client, "tools/call", {
            "name": "search_kevsrobots",
            "arguments": {"query": "motors", "page_type": "lesson", "limit": 3},
        }, request_id=5).json()
        assert "type: lesson" in body["result"]["content"][0]["text"]

    def test_search_tool_handles_no_results(self, client):
        body = self._rpc(client, "tools/call", {
            "name": "search_kevsrobots",
            "arguments": {"query": "zzzqqq", "tag": "nonexistent-tag-xyz"},
        }, request_id=6).json()
        assert "No results" in body["result"]["content"][0]["text"]

    def test_page_tool_accepts_a_full_url(self, client):
        body = self._rpc(client, "tools/call", {
            "name": "get_kevsrobots_page",
            "arguments": {"url": "https://www.kevsrobots.com/learn/micropython/00_intro.html"},
        }, request_id=7).json()
        text = body["result"]["content"][0]["text"]
        assert text.startswith("#")

    def test_page_tool_reports_unknown_pages_clearly(self, client):
        body = self._rpc(client, "tools/call", {
            "name": "get_kevsrobots_page",
            "arguments": {"url": "does/not/exist.html"},
        }, request_id=8).json()
        assert "No indexed content" in body["result"]["content"][0]["text"]

    def test_placeholders_never_reach_the_model(self, client):
        # Empty metadata is stored as the string "none" for Chroma's sake; the
        # model must never be told the course is literally called "none".
        body = self._rpc(client, "tools/call", {
            "name": "search_kevsrobots",
            "arguments": {"query": "laser cut robot", "limit": 5},
        }, request_id=9).json()
        assert "course: none" not in body["result"]["content"][0]["text"]
