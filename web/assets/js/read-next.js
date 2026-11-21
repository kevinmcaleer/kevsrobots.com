// "If you liked this" at the bottom of posts (_includes/related_articles.html).
//
// Asks Nibsy (POST /api/recommendations) for three posts and a course that
// fit this page and what the reader has already read or learned: their
// account history when signed in (the login cookie), plus the history kept
// in this browser (nibsy-history.js). The page ships with tag-related posts
// already in place, so if Nibsy is unreachable nothing changes.
(function () {
    const NIBSY_API = 'https://nibsy.kevsrobots.com';
    const box = document.querySelector('[data-read-next]');
    if (!box || !window.fetch) return;
    const postsEl = box.querySelector('[data-read-next-posts]');
    const courseEl = box.querySelector('[data-read-next-course]');
    const whyEl = box.querySelector('[data-read-next-why]');
    const page = box.dataset.page || window.location.pathname;

    const sid = (() => {
        try {
            let id = sessionStorage.getItem('nibsy-sid');
            if (!id) {
                id = crypto.randomUUID ? crypto.randomUUID() : Math.random().toString(36).slice(2);
                sessionStorage.setItem('nibsy-sid', id);
            }
            return id;
        } catch (e) {
            return 'anon';
        }
    })();

    function beacon(path, data) {
        try {
            navigator.sendBeacon(NIBSY_API + path, new Blob([JSON.stringify(data)], { type: 'application/json' }));
        } catch (e) { /* non-critical */ }
    }

    function el(tag, className, text) {
        const node = document.createElement(tag);
        if (className) node.className = className;
        if (text) node.textContent = text;
        return node;
    }

    function icon(classes) {
        const i = el('i', classes);
        i.setAttribute('aria-hidden', 'true');
        return i;
    }

    function niceDate(iso) {
        if (!iso) return '';
        const d = new Date(iso + 'T00:00:00');
        return isNaN(d) ? '' : d.toLocaleDateString('en-GB', { day: 'numeric', month: 'short', year: '2-digit' });
    }

    // Same markup as the compact /blog/ card (blog_listing_card.html).
    function postCard(rec) {
        const col = el('div', 'col blog-card-col');
        const card = el('article', 'card blog-card bg-light rounded-3 border-0 h-100');
        if (rec.cover) {
            const media = el('div', 'blog-card-media');
            const img = el('img', 'card-img-top');
            img.src = rec.cover;
            img.alt = rec.title;
            img.loading = 'lazy';
            media.appendChild(img);
            card.appendChild(media);
        }
        const body = el('div', 'card-body');
        const title = el('p', 'blog-card-title black text-truncate-2-lines');
        const link = el('a', 'stretched-link text-decoration-none black', rec.title);
        link.href = rec.url;
        link.dataset.nibsyId = rec.content_id;
        title.appendChild(link);
        body.appendChild(title);

        const kind = el('div', 'blog-card-kind small dark-grey mb-2');
        const read = el('span');
        read.append(icon('fa-solid fa-book-open'), ' Read');
        kind.appendChild(read);
        if (rec.video_id) {
            const watch = el('a', 'blog-card-play');
            watch.href = 'https://www.youtube.com/watch?v=' + encodeURIComponent(rec.video_id);
            watch.target = '_blank';
            watch.rel = 'noopener';
            watch.setAttribute('aria-label', 'Watch the video for ' + rec.title + ' on YouTube');
            watch.dataset.nibsyId = rec.content_id;
            watch.dataset.nibsyVideo = '1';
            watch.append(icon('fa-brands fa-youtube'), ' Watch');
            kind.appendChild(watch);
        }
        body.appendChild(kind);

        if (rec.description) body.appendChild(el('p', 'small dark-grey blog-card-desc text-truncate-3-lines', rec.description));
        if (rec.reason) {
            const why = el('p', 'read-next-reason small my-1');
            why.append(icon('fa-solid fa-wand-magic-sparkles'), ' ' + rec.reason);
            body.appendChild(why);
        }
        const date = niceDate(rec.date);
        if (date) {
            const p = el('p', 'small dark-grey my-1');
            p.append(icon('fa-solid fa-calendar-days'), ' ' + date);
            body.appendChild(p);
        }
        card.appendChild(body);
        col.appendChild(card);
        return col;
    }

    function courseState(url) {
        const key = url.substring(0, url.lastIndexOf('/') + 1);
        try {
            const data = JSON.parse(localStorage.getItem('kr-course:' + key));
            if (!data || typeof data.visited !== 'object') return null;
            if (data.completed) return 'done';
            return Object.keys(data.visited).some(id => data.visited[id] && id !== 'index') ? 'started' : null;
        } catch (e) {
            return null;
        }
    }

    // One course: not one they finished, and not one already offered in
    // "Learn the skills for this build" at the top of the post.
    function chooseCourse(courses) {
        const shown = new Set(Array.from(document.querySelectorAll('[data-learn-skills-list] a')).map(a => a.getAttribute('href')));
        const open = courses.filter(c => courseState(c.url) !== 'done');
        return open.find(c => !shown.has(c.url)) || open[0] || null;
    }

    function courseCard(course) {
        const state = courseState(course.url);
        const card = el('a', 'read-next-course');
        card.href = course.url;
        card.dataset.nibsyId = course.content_id;
        if (course.cover) {
            const img = el('img');
            img.src = course.cover;
            img.alt = '';
            img.loading = 'lazy';
            card.appendChild(img);
        }
        const text = el('span', 'read-next-course-body');
        const eyebrow = el('span', 'read-next-eyebrow');
        eyebrow.append(icon('fa-solid fa-graduation-cap'), state === 'started' ? ' Carry on learning' : ' Learn it step by step');
        text.appendChild(eyebrow);
        text.appendChild(el('b', null, course.title));
        const meta = [course.level ? course.level[0].toUpperCase() + course.level.slice(1) : '', course.duration, 'Free course']
            .filter(Boolean).join(' · ');
        text.appendChild(el('span', 'read-next-course-meta', meta));
        if (course.description) text.appendChild(el('span', 'read-next-course-desc', course.description));
        card.appendChild(text);
        card.appendChild(el('span', 'btn btn-sm btn-dark read-next-course-btn', state === 'started' ? 'Carry on' : 'Start the course'));
        return card;
    }

    const history = window.NibsyHistory ? window.NibsyHistory.list().filter(e => e.page !== page) : [];
    const learned = window.NibsyHistory ? window.NibsyHistory.courses() : [];
    const seen = history.concat(learned).sort((a, b) => b.at - a.at).slice(0, 100);

    fetch(NIBSY_API + '/api/recommendations', {
        method: 'POST',
        credentials: 'include',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ page: page, limit: 3, types: ['post'], courses: 3, seen: seen }),
    })
        .then(res => (res.ok ? res.json() : null))
        .then(body => {
            if (!body) return;
            const recs = (body.recommendations || []).slice(0, 3);
            if (recs.length === 3) {
                postsEl.replaceChildren(...recs.map(postCard));
                box.dataset.source = 'nibsy';
            }
            if (whyEl && body.personalised) {
                whyEl.textContent = 'Nibsy picked these from what you’ve read and learned on the site.';
                whyEl.hidden = false;
            }
            const course = chooseCourse(body.courses || []);
            if (course && courseEl) {
                courseEl.replaceChildren(courseCard(course));
                courseEl.hidden = false;
            }
            box.hidden = false;
            const ids = recs.map(r => r.content_id).concat(course ? [course.content_id] : []);
            if (ids.length) beacon('/api/track/impression', { content_ids: ids, source_page: page, session_id: sid });
        })
        .catch(() => {});

    box.addEventListener('click', e => {
        const link = e.target.closest('[data-nibsy-id]');
        if (!link) return;
        beacon('/api/track/click', {
            content_id: parseInt(link.dataset.nibsyId, 10),
            content_url: link.getAttribute('href'),
            source_page: page,
            session_id: sid,
        });
    });
})();
