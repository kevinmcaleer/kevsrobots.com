/*
 * Regression tests for the search results page's engine fallback.
 *
 * The semantic engine ships in a different Docker image to the page that calls
 * it, and the two deploy independently. A page newer than its API is therefore
 * a NORMAL state, not an edge case - and when it happened in production the
 * page rendered nothing at all, with no error, because the fallback only
 * checked for a structured "unavailable" reply and not for the endpoint simply
 * not existing.
 *
 * These run the real fetchResults() extracted from the page against stubbed
 * responses. No network, so they are deterministic.
 *
 *   node tests/frontend/test_fallback.js
 */

const fs = require('fs');
const path = require('path');
const assert = require('assert');

const PAGE = path.join(__dirname, '..', '..', '..', '..', 'web', 'search-results.html');

function loadFetchResults(fetchStub) {
    const html = fs.readFileSync(PAGE, 'utf8');
    const block = html.split('// --- Fetch ---')[1].split('// --- Search box ---')[0];

    const state = {
        rendered: undefined,
        totalCount: undefined,
        container: { innerHTML: '' },
        invite: { style: { display: 'none' } },
        warnings: [],
    };
    const els = {
        'results-container': state.container,
        'total_count': { set textContent(v) { state.totalCount = v; }, get textContent() { return state.totalCount; } },
        'search-time': { textContent: '' },
        'mcp-invite': state.invite,
    };

    const sandbox = {
        fetch: fetchStub,
        console: { warn: (...a) => state.warnings.push(a.join(' ')), error: () => {}, log: () => {} },
        document: { getElementById: (id) => els[id] || { innerHTML: '', textContent: '' } },
        activeRawTypes: () => null,
        renderFilterBar: () => {},
        displayResults: (r) => { state.rendered = r; },
        createPagination: () => {},
        Array, Promise, Error, encodeURIComponent, setTimeout,
    };

    const factory = new Function(...Object.keys(sandbox), block + '\nreturn fetchResults;');
    return { fetchResults: factory(...Object.values(sandbox)), state };
}

const ok = (body) => () => Promise.resolve({
    ok: true, status: 200, json: () => Promise.resolve(body),
});
const status = (code, body) => () => Promise.resolve({
    ok: false, status: code, json: () => Promise.resolve(body),
});
const notJson = () => () => Promise.resolve({
    ok: true, status: 200, json: () => Promise.reject(new SyntaxError('Unexpected token <')),
});

const GOOD_KEYWORD = { results: [{ url: 'a.html' }, { url: 'b.html' }], total_count: 2, page_size: 10, page: 1, facets: {}, execution_time: 0.1 };
const GOOD_SEMANTIC = { results: [{ url: 'x.html' }], total_count: 1, page_size: 10, page: 1, facets: {}, engine: 'hybrid', execution_time: 0.2 };

function routed(handlers) {
    return (url) => {
        // /health is the MCP capability probe, not a search - route it
        // separately or it counts as a keyword call.
        if (url.includes('/health')) {
            return handlers.health
                ? handlers.health(url)
                : Promise.resolve({ ok: true, status: 200, json: () => Promise.resolve({}) });
        }
        const which = url.includes('/search/semantic') ? 'semantic' : 'keyword';
        return handlers[which](url);
    };
}

const tests = [];
const test = (name, fn) => tests.push([name, fn]);

test('semantic 404 (page newer than API) falls back to keyword', async () => {
    // The exact production failure: the old v2.0.0 API has no /search/semantic
    // and answers 404 with a JSON body, which parses fine.
    const { fetchResults, state } = loadFetchResults(routed({
        semantic: status(404, { detail: 'Not Found' }),
        keyword: ok(GOOD_KEYWORD),
    }));
    await fetchResults('robot', 1, 'relevance');
    await new Promise(r => setTimeout(r, 10));
    assert.strictEqual(state.rendered.length, 2, 'should render keyword results');
    assert.ok(!state.container.innerHTML.includes('went wrong'), 'no error banner');
});

test('semantic 500 falls back to keyword', async () => {
    const { fetchResults, state } = loadFetchResults(routed({
        semantic: status(500, { detail: 'boom' }),
        keyword: ok(GOOD_KEYWORD),
    }));
    await fetchResults('robot', 1, 'relevance');
    await new Promise(r => setTimeout(r, 10));
    assert.strictEqual(state.rendered.length, 2);
});

test('non-JSON response falls back to keyword', async () => {
    // An nginx or Cloudflare HTML error page, not JSON.
    const { fetchResults, state } = loadFetchResults(routed({
        semantic: notJson(),
        keyword: ok(GOOD_KEYWORD),
    }));
    await fetchResults('robot', 1, 'relevance');
    await new Promise(r => setTimeout(r, 10));
    assert.strictEqual(state.rendered.length, 2);
});

test('200 with no results array falls back to keyword', async () => {
    const { fetchResults, state } = loadFetchResults(routed({
        semantic: ok({ detail: 'Not Found' }),
        keyword: ok(GOOD_KEYWORD),
    }));
    await fetchResults('robot', 1, 'relevance');
    await new Promise(r => setTimeout(r, 10));
    assert.strictEqual(state.rendered.length, 2, 'a payload with no results array is not usable');
});

test('API reporting its vector index unavailable falls back', async () => {
    const { fetchResults, state } = loadFetchResults(routed({
        semantic: ok({ results: [], engine: 'unavailable', error: 'no index' }),
        keyword: ok(GOOD_KEYWORD),
    }));
    await fetchResults('robot', 1, 'relevance');
    await new Promise(r => setTimeout(r, 10));
    assert.strictEqual(state.rendered.length, 2);
});

test('working semantic engine is used and keyword is not called', async () => {
    let keywordCalls = 0;
    const { fetchResults, state } = loadFetchResults(routed({
        semantic: ok(GOOD_SEMANTIC),
        keyword: () => { keywordCalls++; return ok(GOOD_KEYWORD)(); },
    }));
    await fetchResults('robot', 1, 'relevance');
    await new Promise(r => setTimeout(r, 10));
    assert.strictEqual(state.rendered.length, 1, 'should render semantic results');
    assert.strictEqual(keywordCalls, 0, 'should not also call keyword search');
});

test('"Most Recent" sort uses keyword directly', async () => {
    let semanticCalls = 0;
    const { fetchResults, state } = loadFetchResults(routed({
        semantic: () => { semanticCalls++; return ok(GOOD_SEMANTIC)(); },
        keyword: ok(GOOD_KEYWORD),
    }));
    await fetchResults('robot', 1, 'recent');
    await new Promise(r => setTimeout(r, 10));
    assert.strictEqual(semanticCalls, 0, 'date sort is keyword-only');
    assert.strictEqual(state.rendered.length, 2);
});

test('both engines failing shows an error, not a blank page', async () => {
    const { fetchResults, state } = loadFetchResults(routed({
        semantic: status(404, {}),
        keyword: status(503, {}),
    }));
    await fetchResults('robot', 1, 'relevance');
    await new Promise(r => setTimeout(r, 20));
    assert.ok(state.container.innerHTML.includes('went wrong'), 'user must see an error');
});

test('never renders undefined results', async () => {
    // The shipped bug: displayResults(undefined) and total_count "undefined".
    const { fetchResults, state } = loadFetchResults(routed({
        semantic: status(404, { detail: 'Not Found' }),
        keyword: ok(GOOD_KEYWORD),
    }));
    await fetchResults('robot', 1, 'relevance');
    await new Promise(r => setTimeout(r, 10));
    assert.notStrictEqual(state.rendered, undefined, 'displayResults must never get undefined');
    assert.notStrictEqual(state.totalCount, undefined, 'total_count must never read "undefined"');
});

test('MCP invite stays hidden when the API does not serve MCP', async () => {
    const { fetchResults, state } = loadFetchResults(routed({
        health: ok({ status: 'healthy', version: '2.0.0' }),   // old API, no mcp field
        semantic: status(404, {}),
        keyword: ok(GOOD_KEYWORD),
    }));
    await fetchResults('robot', 1, 'relevance');
    await new Promise(r => setTimeout(r, 20));
    assert.strictEqual(state.invite.style.display, 'none', 'must not advertise an absent connector');
});

test('MCP invite appears when the API reports it mounted', async () => {
    const { fetchResults, state } = loadFetchResults(routed({
        health: ok({ status: 'healthy', mcp: 'mounted' }),
        semantic: ok(GOOD_SEMANTIC),
        keyword: ok(GOOD_KEYWORD),
    }));
    await fetchResults('robot', 1, 'relevance');
    await new Promise(r => setTimeout(r, 20));
    assert.strictEqual(state.invite.style.display, '', 'should be revealed');
});

(async () => {
    let failed = 0;
    for (const [name, fn] of tests) {
        try {
            await fn();
            console.log(`  ok   ${name}`);
        } catch (error) {
            failed++;
            console.log(`  FAIL ${name}\n       ${error.message}`);
        }
    }
    console.log(`\n${tests.length - failed} passed, ${failed} failed`);
    process.exit(failed ? 1 : 0);
})();
