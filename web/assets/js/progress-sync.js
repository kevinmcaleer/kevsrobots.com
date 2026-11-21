// Course progress for signed-in learners, saved to their account (#633).
//
// course-progress.js keeps each course's progress in the browser
// (localStorage "kr-course:/learn/<course>/"). For a signed-in reader this
// script also sends those records to Nibsy, which merges them with what
// other devices sent, and writes the merged result back to the browser. So
// learn-progress.js and course-progress.js keep reading localStorage and
// work the same whether or not the reader is signed in.
//
// window.krProgressSync:
//   signedIn()          true when the Chatter "username" cookie is set
//   syncAll()           send every course in this browser (on the first
//                       visit after signing in, then at most every few
//                       minutes); resolves when the browser is up to date
//   syncCourse(key)     send one course (after a lesson or quiz answer)
//   reset(key)          forget one course on the account too
(function () {
    const NIBSY_API = 'https://nibsy.kevsrobots.com';
    const STORAGE_PREFIX = 'kr-course:';
    const ALL_EVERY_MS = 5 * 60 * 1000;

    function username() {
        const match = document.cookie.match(/(?:^|;\s*)username=([^;]+)/);
        return match ? decodeURIComponent(match[1]) : null;
    }

    function session(key, value) {
        try {
            if (value === undefined) return sessionStorage.getItem(key);
            sessionStorage.setItem(key, value);
        } catch (e) { /* storage blocked */ }
        return null;
    }

    function local(key, value) {
        try {
            if (value === undefined) return localStorage.getItem(key);
            localStorage.setItem(key, value);
        } catch (e) { /* storage blocked */ }
        return null;
    }

    // The login cookie can outlive the token; stop trying for this visit.
    function signedIn() {
        return !!username() && session('kr-progress-off') !== '1';
    }

    function readLocal(key) {
        try {
            const data = JSON.parse(local(STORAGE_PREFIX + key));
            if (data && typeof data === 'object') return data;
        } catch (e) { /* corrupt */ }
        return null;
    }

    function toServer(key, data) {
        return {
            course: key,
            visited: Object.keys(data.visited || {}).filter(id => data.visited[id]),
            answers: data.answers || {},
            last: data.last || null,
            updated: data.updated || 0,
            completed: data.completed || null
        };
    }

    // Same rules as the server: lessons are a union, the first answer to a
    // question stands, the newest "last" lesson wins, the first completion is kept.
    function writeBack(records) {
        (records || []).forEach(rec => {
            const data = readLocal(rec.course) || {};
            const visited = data.visited || {};
            rec.visited.forEach(id => { visited[id] = true; });
            const answers = Object.assign({}, data.answers || {}, rec.answers);
            const merged = Object.assign({}, data, { visited, answers });
            if (rec.last && (!data.last || (rec.updated || 0) >= (data.updated || 0))) merged.last = rec.last;
            merged.updated = Math.max(data.updated || 0, rec.updated || 0);
            if (rec.completed && (!data.completed || rec.completed < data.completed)) merged.completed = rec.completed;
            local(STORAGE_PREFIX + rec.course, JSON.stringify(merged));
        });
    }

    function request(method, path, body) {
        if (!signedIn()) return Promise.resolve(null);
        const options = { method, credentials: 'include' };
        if (body) {
            options.headers = { 'Content-Type': 'application/json' };
            options.body = JSON.stringify(body);
        }
        return fetch(NIBSY_API + path, options)
            .then(res => {
                if (res.status === 401) session('kr-progress-off', '1');
                return res.ok ? res.json() : null;
            })
            .catch(() => null);
    }

    function send(courses) {
        return request('POST', '/api/me/progress', { courses }).then(body => {
            if (body) writeBack(body.courses);
            return body;
        });
    }

    function syncAll() {
        const user = username();
        if (!signedIn()) return Promise.resolve(null);
        const importedKey = 'kr-progress-imported:' + user;
        const last = Number(session('kr-progress-all:' + user)) || 0;
        if (local(importedKey) === '1' && Date.now() - last < ALL_EVERY_MS) return Promise.resolve(null);

        const courses = [];
        try {
            for (let i = 0; i < localStorage.length; i++) {
                const key = localStorage.key(i);
                if (!key || !key.startsWith(STORAGE_PREFIX)) continue;
                const data = readLocal(key.slice(STORAGE_PREFIX.length));
                if (data) courses.push(toServer(key.slice(STORAGE_PREFIX.length), data));
            }
        } catch (e) { /* storage blocked */ }
        return send(courses).then(body => {
            if (body) {
                local(importedKey, '1');
                session('kr-progress-all:' + user, String(Date.now()));
            }
            return body;
        });
    }

    // Lesson pages call this after every change; wait for a quiet moment.
    let timer = null;
    let pending = null;
    function syncCourse(key) {
        if (!signedIn()) return Promise.resolve(null);
        clearTimeout(timer);
        if (pending) pending(null);
        return new Promise(resolve => {
            pending = resolve;
            timer = setTimeout(() => {
                const done = pending;
                pending = null;
                const data = readLocal(key);
                const first = local('kr-progress-imported:' + username()) !== '1';
                (first ? syncAll() : send(data ? [toServer(key, data)] : [])).then(done);
            }, 800);
        });
    }

    function reset(key) {
        return request('DELETE', '/api/me/progress?course=' + encodeURIComponent(key));
    }

    window.krProgressSync = { signedIn, syncAll, syncCourse, reset };
})();
