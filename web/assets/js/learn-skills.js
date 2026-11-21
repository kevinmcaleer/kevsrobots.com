// "Learn the skills for this build" (#636, _includes/learn_skills.html):
// up to two courses from Nibsy that teach what this post or project uses.
// Each card says whether the reader has started or finished the course,
// from their saved progress (course-progress.js).
(function () {
    const NIBSY_API = 'https://nibsy.kevsrobots.com';
    const box = document.querySelector('[data-learn-skills]');
    if (!box) return;
    const list = box.querySelector('[data-learn-skills-list]');

    function progress(url) {
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

    function el(tag, className, text) {
        const node = document.createElement(tag);
        if (className) node.className = className;
        if (text) node.textContent = text;
        return node;
    }

    fetch(NIBSY_API + '/api/learn/for-page?limit=2&page=' + encodeURIComponent(box.dataset.page), { credentials: 'include' })
        .then(res => (res.ok ? res.json() : null))
        .then(body => {
            if (!body || !Array.isArray(body.courses) || body.courses.length === 0) return;
            let suggested = false;
            body.courses.forEach(course => {
                const card = el('a', 'learn-skills-card');
                card.href = course.url;
                if (course.cover) {
                    const img = el('img');
                    img.src = course.cover;
                    img.alt = '';
                    img.loading = 'lazy';
                    card.appendChild(img);
                }
                const text = el('span', 'learn-skills-body');
                text.appendChild(el('b', null, course.title));
                const meta = [course.level ? course.level[0].toUpperCase() + course.level.slice(1) : '', course.duration]
                    .filter(Boolean).join(' · ');
                if (meta) text.appendChild(el('span', 'learn-skills-meta', meta));
                const state = progress(course.url);
                if (state === 'done') {
                    text.appendChild(el('span', 'learn-skills-state is-done', 'You finished this'));
                } else if (state === 'started') {
                    text.appendChild(el('span', 'learn-skills-state', 'Carry on with this'));
                } else if (!suggested) {
                    suggested = true;
                    text.appendChild(el('span', 'learn-skills-state', course.reason === 'Recommended by Kev' ? 'Picked by Kev' : 'Start here'));
                }
                card.appendChild(text);
                list.appendChild(card);
            });
            box.hidden = false;
        })
        .catch(() => {});
})();
