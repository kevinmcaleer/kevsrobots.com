// My Learning (/learn/me/, #634).
//
// Draws the page from the reader's course progress in localStorage
// ("kr-course:/learn/<course>/", written by course-progress.js) and the
// course and pathway list the page carries (#my-learning-data). For a
// signed-in reader, progress-sync.js first brings in progress from their
// account, and the page is drawn again with it. "Build next" cards come
// from Nibsy (/api/learn/follow-on).
(function () {
    const NIBSY_API = 'https://nibsy.kevsrobots.com';
    const STORAGE_PREFIX = 'kr-course:';
    const root = document.getElementById('my-learning');
    const dataEl = document.getElementById('my-learning-data');
    if (!root || !dataEl) return;

    const data = JSON.parse(dataEl.textContent);
    const courses = {};
    data.courses.forEach(c => { courses[c.key] = c; });

    const $ = (sel) => root.querySelector(sel);

    function el(tag, className, text) {
        const node = document.createElement(tag);
        if (className) node.className = className;
        if (text !== undefined && text !== null) node.textContent = text;
        return node;
    }

    function cover(src, alt) {
        if (!src) return el('span', 'ml-cover-blank');
        const img = el('img');
        img.src = src;
        img.alt = alt || '';
        img.loading = 'lazy';
        return img;
    }

    function minutes(duration) {
        const m = /(?:(\d+)h)?\s*(?:(\d+)m)?/.exec(duration || '');
        return m ? (Number(m[1]) || 0) * 60 + (Number(m[2]) || 0) : 0;
    }

    function formatMinutes(total) {
        const h = Math.floor(total / 60);
        const m = Math.round(total % 60);
        return h ? `${h}h ${m}m` : `${m}m`;
    }

    function formatDate(ms) {
        return new Date(ms).toLocaleDateString(undefined, { day: 'numeric', month: 'short', year: 'numeric' });
    }

    function lessonNumber(id) {
        const m = /^(\d+)/.exec(id || '');
        return m ? Number(m[1]) : null;
    }

    // { key: { course, visited, percent, done, last, updated, completed, right, answered } }
    function readProgress() {
        const progress = {};
        try {
            for (let i = 0; i < localStorage.length; i++) {
                const key = localStorage.key(i);
                if (!key || !key.startsWith(STORAGE_PREFIX)) continue;
                const course = courses[key.slice(STORAGE_PREFIX.length)];
                const rec = JSON.parse(localStorage.getItem(key));
                if (!course || !rec || typeof rec.visited !== 'object') continue;
                const visited = Object.keys(rec.visited).filter(id => rec.visited[id] && id !== 'index' && id !== '').length;
                if (visited === 0) continue;
                const lessons = course.lessons || visited;
                const answers = Object.values(rec.answers || {});
                const done = visited >= lessons || !!rec.completed;
                progress[course.key] = {
                    course,
                    visited: Math.min(visited, lessons),
                    percent: done ? 100 : Math.round((visited / lessons) * 100),
                    done,
                    last: typeof rec.last === 'string' ? rec.last : '',
                    updated: Number(rec.updated) || 0,
                    completed: Number(rec.completed) || Number(rec.updated) || 0,
                    right: answers.filter(Boolean).length,
                    answered: answers.length
                };
            }
        } catch (e) { /* storage blocked or corrupt */ }
        return progress;
    }

    function quizText(p) {
        return p.answered ? ` · quiz ${p.right}/${p.answered}` : '';
    }

    function resumeHref(p) {
        return p.last ? p.course.key + p.last + '.html' : p.course.link;
    }

    function bar(percent) {
        const outer = el('span', 'ml-bar');
        const fill = el('span');
        fill.style.width = percent + '%';
        outer.appendChild(fill);
        outer.setAttribute('role', 'progressbar');
        outer.setAttribute('aria-valuenow', String(percent));
        outer.setAttribute('aria-valuemin', '0');
        outer.setAttribute('aria-valuemax', '100');
        return outer;
    }

    function show(sel, visible) {
        const node = $(sel);
        if (node) node.hidden = !visible;
    }

    function renderResume(current) {
        const box = $('[data-ml-resume]');
        box.textContent = '';
        if (!current) { box.hidden = true; return; }
        const c = current.course;
        box.appendChild(cover(c.cover, c.name));
        const body = el('div', 'ml-resume-body');
        body.appendChild(el('span', 'ml-kicker', 'Continue where you left off'));
        body.appendChild(el('h2', null, c.name));
        const n = lessonNumber(current.last);
        const left = Math.max(0, minutes(c.duration) * (1 - current.visited / (c.lessons || 1)));
        const bits = [`${current.visited} of ${c.lessons} lessons`];
        if (n !== null) bits.push(`last opened lesson ${n}`);
        if (left >= 1) bits.push(`about ${formatMinutes(left)} left`);
        body.appendChild(el('p', null, bits.join(' · ')));
        body.appendChild(bar(current.percent));
        box.appendChild(body);
        const go = el('a', 'btn btn-primary', 'Resume');
        go.href = resumeHref(current);
        box.appendChild(go);
        box.hidden = false;
    }

    function startedPaths(progress) {
        return data.pathways
            .map(path => {
                const steps = path.courses.map(step => ({ ...step, progress: step.key ? progress[step.key] : null }));
                const real = steps.filter(s => s.key && courses[s.key]);
                const done = real.filter(s => s.progress && s.progress.done);
                const started = real.filter(s => s.progress);
                const latest = Math.max(0, ...started.map(s => s.progress.updated));
                return { path, steps, real, done, started, latest };
            })
            .filter(p => p.started.length > 0)
            .sort((a, b) => b.latest - a.latest);
    }

    function renderPaths(paths) {
        const holder = $('[data-ml-paths]');
        holder.textContent = '';
        paths.forEach(({ path, steps, real, done }) => {
            const row = el('a', 'ml-path');
            row.href = path.url;
            const head = el('span', 'ml-path-head');
            head.appendChild(el('b', null, path.title));
            head.appendChild(el('span', null, `${done.length} of ${real.length} courses`));
            row.appendChild(head);

            const segs = el('span', 'ml-segs');
            steps.forEach(step => {
                const seg = el('span', 'ml-seg');
                if (step.soon) seg.classList.add('is-soon');
                else if (step.progress && step.progress.done) seg.classList.add('is-done');
                else if (step.progress) seg.classList.add('is-started');
                seg.title = step.soon ? `${step.soon} (coming soon)` : (courses[step.key] || {}).name || '';
                segs.appendChild(seg);
            });
            row.appendChild(segs);

            const now = real.find(s => s.progress && !s.progress.done);
            const next = real.find(s => !s.progress);
            const notes = [];
            if (now) notes.push('Now: ' + courses[now.key].name);
            if (next) notes.push('Next: ' + courses[next.key].name);
            if (!next && !now) notes.push('Pathway finished');
            row.appendChild(el('span', 'ml-path-note', notes.join(' · ')));
            holder.appendChild(row);
        });
    }

    function renderCourses(inProgress) {
        const holder = $('[data-ml-courses]');
        holder.textContent = '';
        inProgress.forEach(p => {
            const row = el('a', 'ml-course');
            row.href = resumeHref(p);
            row.appendChild(cover(p.course.cover));
            const body = el('span', 'ml-course-body');
            const head = el('span', 'ml-course-head');
            head.appendChild(el('b', null, p.course.name));
            head.appendChild(el('span', null, `${p.visited} of ${p.course.lessons}${quizText(p)}`));
            body.appendChild(head);
            body.appendChild(bar(p.percent));
            row.appendChild(body);
            holder.appendChild(row);
        });
    }

    function nextCard(href, img, kicker, title, meta) {
        const card = el('a', 'ml-next-card');
        card.href = href;
        card.appendChild(cover(img));
        const body = el('span', 'ml-next-body');
        body.appendChild(el('span', 'ml-kicker', kicker));
        body.appendChild(el('b', null, title));
        if (meta) body.appendChild(el('span', 'ml-meta', meta));
        card.appendChild(body);
        return card;
    }

    function courseMeta(c) {
        return [c.level ? c.level[0].toUpperCase() + c.level.slice(1) : '', c.duration, c.lessons ? `${c.lessons} lessons` : '']
            .filter(Boolean).join(' · ');
    }

    let projectsFor = null;
    let projectCards = [];

    function renderNext(paths, progress, finished) {
        const holder = $('[data-ml-next]');
        holder.textContent = '';
        const picked = new Set();
        let count = 0;
        paths.forEach(({ path, real }) => {
            if (count >= 3) return;
            const next = real.find(s => !s.progress && !picked.has(s.key));
            if (!next) return;
            picked.add(next.key);
            const c = courses[next.key];
            holder.appendChild(nextCard(c.link, c.cover, `Next in ${path.title}`, c.name, courseMeta(c)));
            count++;
        });
        projectCards.slice(0, Math.max(0, 3 - count)).forEach(p => {
            holder.appendChild(nextCard(p.url, p.cover, p.reason || 'Build next', p.title, ''));
            count++;
        });
        show('[data-ml-next-section]', count > 0);

        // Projects to build after the course finished most recently.
        const basis = finished[0];
        if (basis && projectsFor !== basis.course.key && count < 3) {
            projectsFor = basis.course.key;
            fetch(NIBSY_API + '/api/learn/follow-on?limit=3&course=' + encodeURIComponent(basis.course.link))
                .then(res => (res.ok ? res.json() : null))
                .then(body => {
                    if (!body || !Array.isArray(body.projects)) return;
                    projectCards = body.projects.map(p => ({
                        ...p,
                        reason: p.reason === 'Recommended by Kev' ? `Build next, after ${basis.course.name}` : `Because you finished ${basis.course.name}`
                    }));
                    if (projectCards.length) render();
                })
                .catch(() => {});
        }
    }

    function renderDone(finished) {
        const holder = $('[data-ml-done]');
        holder.textContent = '';
        finished.forEach(p => {
            const row = el('a', 'ml-done-card');
            row.href = p.course.link;
            row.appendChild(cover(p.course.cover));
            const body = el('span', 'ml-done-body');
            body.appendChild(el('b', null, p.course.name));
            body.appendChild(el('span', 'ml-meta', formatDate(p.completed) + quizText(p)));
            row.appendChild(body);
            holder.appendChild(row);
        });
    }

    function render() {
        const progress = readProgress();
        const all = Object.values(progress);
        const inProgress = all.filter(p => !p.done).sort((a, b) => b.updated - a.updated);
        const finished = all.filter(p => p.done).sort((a, b) => b.completed - a.completed);
        const paths = startedPaths(progress);
        const signedIn = window.krProgressSync && window.krProgressSync.signedIn();

        show('[data-ml-signin]', !signedIn);
        show('[data-ml-empty]', all.length === 0);
        show('[data-ml-stats]', all.length > 0);
        show('[data-ml-clear-wrap]', all.length > 0);

        const learnt = all.reduce((sum, p) => sum + minutes(p.course.duration) * (p.percent / 100), 0);
        $('[data-ml-stat="finished"]').textContent = String(finished.length);
        $('[data-ml-stat="paths"]').textContent = String(paths.length);
        $('[data-ml-stat="time"]').textContent = formatMinutes(learnt);

        renderResume(inProgress[0]);
        renderPaths(paths);
        show('[data-ml-paths-section]', paths.length > 0);
        renderCourses(inProgress);
        show('[data-ml-courses-section]', inProgress.length > 0);
        renderNext(paths, progress, finished);
        renderDone(finished);
        show('[data-ml-done-section]', finished.length > 0);
    }

    $('[data-ml-clear]').addEventListener('click', () => {
        const signedIn = window.krProgressSync && window.krProgressSync.signedIn();
        const where = signedIn ? 'this browser and your account' : 'this browser';
        if (!confirm(`Clear the progress and quiz scores of every course from ${where}?`)) return;
        try {
            Object.keys(localStorage)
                .filter(k => k.startsWith(STORAGE_PREFIX))
                .forEach(k => localStorage.removeItem(k));
        } catch (e) { /* storage blocked */ }
        const done = signedIn
            ? fetch(NIBSY_API + '/api/me/progress', { method: 'DELETE', credentials: 'include' }).catch(() => null)
            : Promise.resolve();
        projectCards = [];
        projectsFor = null;
        done.then(render);
    });

    render();
    if (window.krProgressSync && window.krProgressSync.signedIn()) {
        window.krProgressSync.syncAll({ force: true }).then(body => { if (body) render(); });
    }
})();
