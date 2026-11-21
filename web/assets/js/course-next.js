// "Build something next" on a course's last lesson (#635,
// _includes/course/next_steps.html): projects from Nibsy that use what the
// course taught, the course's pinned projects first. Stays hidden when
// Nibsy is unreachable or has nothing that fits.
(function () {
    const NIBSY_API = 'https://nibsy.kevsrobots.com';
    const script = document.currentScript;
    const box = document.querySelector('[data-course-projects]');
    if (!script || !box) return;
    const list = box.querySelector('[data-course-projects-list]');

    fetch(NIBSY_API + '/api/learn/follow-on?limit=3&course=' + encodeURIComponent(script.dataset.course))
        .then(res => (res.ok ? res.json() : null))
        .then(body => {
            if (!body || !Array.isArray(body.projects) || body.projects.length === 0) return;
            body.projects.forEach(p => {
                const card = document.createElement('a');
                card.className = 'course-next-project';
                card.href = p.url;
                if (p.cover) {
                    const img = document.createElement('img');
                    img.src = p.cover;
                    img.alt = '';
                    img.loading = 'lazy';
                    card.appendChild(img);
                }
                const title = document.createElement('b');
                title.textContent = p.title;
                card.appendChild(title);
                if (p.reason === 'Recommended by Kev') {
                    const tag = document.createElement('span');
                    tag.className = 'course-next-pin';
                    tag.textContent = 'Picked by Kev';
                    card.appendChild(tag);
                }
                list.appendChild(card);
            });
            box.hidden = false;
        })
        .catch(() => {});
})();
