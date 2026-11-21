// Nibsy reading history kept in this browser (localStorage "nibsy-read").
//
// Every content page a visitor opens is noted here: [{ page, at }], newest
// first, at most 100 pages and 90 days. It never leaves the browser except
// as part of a recommendations request (read-next.js), where the Nibsy API
// uses it for that one answer and stores nothing. Signed-in readers also
// have server-side history (90 days); "Forget What I've Read" in the Nibsy
// menu clears both.
(function () {
    if (window.NibsyHistory) return;

    const KEY = 'nibsy-read';
    const MAX = 100;
    const MAX_AGE = 90 * 86400000;
    // Pages that are never worth recommending from.
    const SKIP = /^\/(admin|account|login|register|reset-password|profile|search|moderator|feedback|thanks)/;

    function list() {
        try {
            const data = JSON.parse(localStorage.getItem(KEY));
            if (!Array.isArray(data)) return [];
            const cutoff = Date.now() - MAX_AGE;
            return data.filter(e => e && typeof e.page === 'string' && e.at > cutoff);
        } catch (e) {
            return [];
        }
    }

    function save(entries) {
        try {
            localStorage.setItem(KEY, JSON.stringify(entries.slice(0, MAX)));
        } catch (e) { /* storage blocked: history just won't persist */ }
    }

    function record(path) {
        if (!path || path === '/' || SKIP.test(path)) return;
        // Section indexes (/blog/, /learn/) say nothing about interests.
        if (path.endsWith('/') && path.split('/').filter(Boolean).length < 2) return;
        const entries = list().filter(e => e.page !== path);
        entries.unshift({ page: path, at: Date.now() });
        save(entries);
    }

    function clear() {
        try { localStorage.removeItem(KEY); } catch (e) { /* nothing to clear */ }
    }

    // Courses the reader has worked through (course-progress.js), as the
    // course folder and when they were last there.
    function courses() {
        const out = [];
        try {
            for (let i = 0; i < localStorage.length; i++) {
                const key = localStorage.key(i);
                if (!key || key.indexOf('kr-course:') !== 0) continue;
                const data = JSON.parse(localStorage.getItem(key));
                if (data && data.updated) out.push({ page: key.slice('kr-course:'.length), at: data.updated });
            }
        } catch (e) { /* storage blocked */ }
        return out;
    }

    window.NibsyHistory = { list, record, clear, courses };
    record(window.location.pathname);
})();
