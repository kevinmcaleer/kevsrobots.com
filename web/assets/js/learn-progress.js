// Lesson progress on the Learn homepage and on learning pathway pages.
//
// course-progress.js saves each reader's progress in their own browser
// (localStorage, one "kr-course:/learn/<course>/" record per course). This
// script reads those records, so it works whether or not the reader is
// logged in, and does nothing for a reader with no progress. For a signed-in
// reader, progress-sync.js first brings in progress from their account
// (other devices), and the page is drawn again if anything came back.
//
//   /learn/           fills the "Continue learning" strip (#learn-continue)
//                     with up to three courses in progress, most recent first.
//   homepage          the same strip, in the Courses band; course names and
//                     covers come from #kr-course-index (home/courses.html).
//   pathway pages     marks each course row finished or in progress, and
//                     says how many courses are finished.
(function () {
    const STORAGE_PREFIX = 'kr-course:';

    // { "/learn/micropython/": { visited: n, last: "03_...", updated: ms } }
    function readProgress() {
        const progress = {};
        try {
            for (let i = 0; i < localStorage.length; i++) {
                const key = localStorage.key(i);
                if (!key || !key.startsWith(STORAGE_PREFIX)) continue;
                const data = JSON.parse(localStorage.getItem(key));
                if (!data || typeof data.visited !== 'object') continue;
                // "index" is a copy of the first lesson, not a lesson of its own
                const visited = Object.keys(data.visited).filter(id => data.visited[id] && id !== 'index' && id !== '').length;
                if (visited === 0) continue;
                progress[key.slice(STORAGE_PREFIX.length)] = {
                    visited: visited,
                    last: typeof data.last === 'string' ? data.last : '',
                    updated: Number(data.updated) || 0
                };
            }
        } catch (e) { /* storage blocked or corrupt: show nothing */ }
        return progress;
    }

    function render() {
    const progress = readProgress();
    if (Object.keys(progress).length === 0) return;

    function el(tag, className, text) {
        const node = document.createElement(tag);
        if (className) node.className = className;
        if (text) node.textContent = text;
        return node;
    }

    // --- Learn homepage: Continue learning -----------------------------------
    const strip = document.getElementById('learn-continue');
    if (strip) {
        const courses = [];
        // The homepage has no course catalogue to read names and covers
        // from, so it gives the same facts as JSON (#kr-course-index).
        const index = document.getElementById('kr-course-index');
        if (index) {
            let list = [];
            try { list = JSON.parse(index.textContent); } catch (e) { list = []; }
            list.forEach(c => {
                const key = c.link.substring(0, c.link.lastIndexOf('/') + 1);
                const saved = progress[key];
                const lessons = Number(c.lessons) || 0;
                if (!saved || lessons === 0 || saved.visited >= lessons) return;
                courses.push({
                    href: saved.last ? key + saved.last + '.html' : c.link,
                    name: c.name,
                    cover: c.cover,
                    visited: saved.visited,
                    lessons: lessons,
                    updated: saved.updated
                });
            });
        }
        document.querySelectorAll('#courses .course-card').forEach(card => {
            const link = card.querySelector('a[href]');
            if (!link) return;
            const href = link.getAttribute('href');
            const key = href.substring(0, href.lastIndexOf('/') + 1);
            const saved = progress[key];
            const lessons = Number(card.dataset.lessons) || 0;
            if (!saved || lessons === 0 || saved.visited >= lessons) return;
            const img = card.querySelector('img');
            const name = card.querySelector('[data-course-name]');
            courses.push({
                href: saved.last ? key + saved.last + '.html' : href,
                name: name ? name.textContent : '',
                cover: img ? img.getAttribute('src') : '',
                visited: saved.visited,
                lessons: lessons,
                updated: saved.updated
            });
        });
        courses.sort((a, b) => b.updated - a.updated || b.visited - a.visited);

        const holder = strip.querySelector('[data-continue-cards]');
        holder.textContent = '';
        courses.slice(0, 3).forEach(course => {
            const percent = Math.round((course.visited / course.lessons) * 100);
            const card = el('a', 'learn-continue-card');
            card.href = course.href;
            if (course.cover) {
                const img = el('img');
                img.src = course.cover;
                img.alt = '';
                card.appendChild(img);
            }
            const body = el('span', 'learn-continue-body');
            body.appendChild(el('span', 'learn-continue-name', course.name));
            body.appendChild(el('span', 'learn-continue-meta', `${course.visited} of ${course.lessons} lessons · ${percent}%`));
            const bar = el('span', 'learn-continue-bar');
            const fill = el('span');
            fill.style.width = percent + '%';
            bar.appendChild(fill);
            body.appendChild(bar);
            card.appendChild(body);
            card.appendChild(el('span', 'btn btn-primary btn-sm', 'Resume'));
            holder.appendChild(card);
        });
        strip.hidden = courses.length === 0;
    }

    // --- Pathway page: finished and in-progress courses ------------------------
    const steps = Array.from(document.querySelectorAll('.pathway-step[data-course-key]'));
    if (steps.length > 0) {
        let finished = 0;
        let started = 0;
        steps.forEach(step => {
            const go = step.querySelector('[data-course-go]');
            if (go && !go.dataset.original) {
                go.dataset.original = JSON.stringify({ text: go.textContent, href: go.getAttribute('href') });
            }
            const saved = progress[step.dataset.courseKey];
            const lessons = Number(step.dataset.lessons) || 0;
            if (!saved || lessons === 0) return;
            started++;
            const done = saved.visited >= lessons;
            if (done) finished++;
            step.classList.remove('is-done', 'is-started');
            step.classList.add(done ? 'is-done' : 'is-started');

            if (go) {
                const original = JSON.parse(go.dataset.original);
                go.href = original.href;
                go.classList.replace('btn-primary', 'btn-outline-dark');
                go.textContent = done ? 'Review' : 'Continue';
                if (!done && saved.last) go.href = step.dataset.courseKey + saved.last + '.html';
                if (!done) go.classList.replace('btn-outline-dark', 'btn-primary');
            }
            const note = step.querySelector('[data-course-progress]');
            if (note) {
                note.textContent = done ? 'Finished' : `${saved.visited} of ${lessons} lessons`;
                note.hidden = false;
            }
            const number = step.querySelector('.pathway-step-number');
            if (number && done) number.textContent = '✓';
        });

        const summary = document.querySelector('[data-pathway-progress]');
        if (summary && started > 0) {
            summary.textContent = `You have finished ${finished} of ${steps.length} courses in this pathway.`;
            summary.hidden = false;
        }
    }
    }

    render();
    if (window.krProgressSync && window.krProgressSync.signedIn()) {
        window.krProgressSync.syncAll().then(body => { if (body) render(); });
    }
})();
