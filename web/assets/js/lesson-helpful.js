// "Was this lesson helpful?" (#637, _includes/lesson_helpful.html).
(function () {
    const NIBSY_API = 'https://nibsy.kevsrobots.com';
    const box = document.querySelector('[data-lesson-helpful]');
    if (!box) return;
    const page = window.location.pathname;
    const key = 'kr-helpful:' + page;
    const thanks = box.querySelector('[data-helpful-thanks]');

    function remembered() {
        try {
            const v = localStorage.getItem(key);
            return v === 'yes' ? true : v === 'no' ? false : null;
        } catch (e) {
            return null;
        }
    }

    function paint(answer) {
        box.querySelectorAll('[data-helpful]').forEach(b => {
            const pressed = answer !== null && (b.dataset.helpful === 'yes') === answer;
            b.setAttribute('aria-pressed', String(pressed));
            b.classList.toggle('is-picked', pressed);
        });
    }

    paint(remembered());

    box.addEventListener('click', (e) => {
        const button = e.target.closest('[data-helpful]');
        if (!button) return;
        const helpful = button.dataset.helpful === 'yes';
        const previous = remembered();
        if (previous === helpful) return;
        try { localStorage.setItem(key, helpful ? 'yes' : 'no'); } catch (err) { /* storage blocked */ }
        paint(helpful);
        thanks.textContent = helpful
            ? 'Thanks! Glad it helped.'
            : 'Thanks for saying. Ask a question in the comments and Kev can improve it.';
        fetch(NIBSY_API + '/api/learn/helpful', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ page, helpful, previous })
        }).catch(() => {});
    });
})();
