// Share buttons for a finished course, and the "Course complete" dialog.
//
// Markup: _includes/course_share_buttons.html and course_complete_dialog.html.
// The share links work without this script. This adds:
//   * the reader's quiz score to the share text, when they answered every quiz
//   * the phone's own Share sheet (navigator.share), when there is one
//   * Copy link
//   * Mastodon: asks for the reader's server once and remembers it
//
// course-progress.js calls window.krCourseShare.open({ score }) the first time
// a course is finished, and when the reader clicks "Share your achievement".
(function () {
    const MASTODON_KEY = 'kr-mastodon-server';

    function storageGet(key) {
        try { return localStorage.getItem(key) || ''; } catch (e) { return ''; }
    }
    function storageSet(key, value) {
        try { localStorage.setItem(key, value); } catch (e) { /* not saved, fine */ }
    }

    function setup(box) {
        const url = box.dataset.shareUrl;
        const name = box.dataset.shareName;
        const baseText = box.dataset.shareText;
        let text = baseText;
        const status = box.querySelector('[data-share-status]');

        function say(message) {
            if (!status) return;
            status.textContent = message;
            clearTimeout(say.timer);
            say.timer = setTimeout(() => { status.textContent = ''; }, 4000);
        }

        function links() {
            const enc = encodeURIComponent;
            return {
                x: `https://x.com/intent/post?text=${enc(text)}&url=${enc(url)}`,
                bluesky: `https://bsky.app/intent/compose?text=${enc(text + ' ' + url)}`,
                linkedin: `https://www.linkedin.com/sharing/share-offsite/?url=${enc(url)}`,
                facebook: `https://www.facebook.com/sharer/sharer.php?u=${enc(url)}`
            };
        }

        function refresh() {
            const hrefs = links();
            box.querySelectorAll('[data-share-to]').forEach(a => {
                if (hrefs[a.dataset.shareTo]) a.href = hrefs[a.dataset.shareTo];
            });
        }

        box.krSetScore = function (score) {
            text = score ? `${baseText}, and scored ${score} on the quizzes` : baseText;
            refresh();
        };

        // Native share sheet (phones, Safari, Edge)
        const native = box.querySelector('[data-share-native]');
        if (native && navigator.share) {
            native.hidden = false;
            native.addEventListener('click', () => {
                navigator.share({ title: `I completed ${name}`, text: text, url: url })
                    .catch(() => { /* cancelled */ });
            });
        }

        // Copy link
        const copy = box.querySelector('[data-share-copy]');
        if (copy) {
            copy.addEventListener('click', () => {
                const value = `${text} ${url}`;
                const done = () => say('Copied! Paste it anywhere you like.');
                if (navigator.clipboard && navigator.clipboard.writeText) {
                    navigator.clipboard.writeText(value).then(done, () => window.prompt('Copy this:', value));
                } else {
                    window.prompt('Copy this:', value);
                }
            });
        }

        // Mastodon: every server has its own /share page, so ask which one
        const mastodon = box.querySelector('[data-share-mastodon]');
        const form = box.querySelector('[data-share-mastodon-form]');
        const input = box.querySelector('[data-share-mastodon-server]');
        if (mastodon && form && input) {
            input.value = storageGet(MASTODON_KEY);
            mastodon.addEventListener('click', () => {
                form.hidden = !form.hidden;
                mastodon.setAttribute('aria-expanded', String(!form.hidden));
                if (!form.hidden) input.focus();
            });
            form.addEventListener('submit', (e) => {
                e.preventDefault();
                let server = (input.value || input.placeholder).trim()
                    .replace(/^https?:\/\//i, '').replace(/^@?[^@\s]+@/, '').replace(/\/.*$/, '');
                if (!/^[a-z0-9.-]+\.[a-z]{2,}$/i.test(server)) {
                    say('That doesn\'t look like a server name, try something like mastodon.social');
                    return;
                }
                storageSet(MASTODON_KEY, server);
                window.open(`https://${server}/share?text=${encodeURIComponent(text + ' ' + url)}`, '_blank', 'noopener');
            });
        }
    }

    const boxes = Array.from(document.querySelectorAll('[data-course-share]'));
    boxes.forEach(setup);

    // --- The "Course complete" dialog --------------------------------------
    const dialog = document.getElementById('kr-course-complete');

    function confetti() {
        const layer = dialog.querySelector('.kr-complete-confetti');
        if (!layer || window.matchMedia('(prefers-reduced-motion: reduce)').matches) return;
        layer.textContent = '';
        const colours = ['#C22032', '#FFCA29', '#EB642B', '#2EA05A', '#3b82f6'];
        for (let i = 0; i < 40; i++) {
            const bit = document.createElement('i');
            bit.style.left = Math.random() * 100 + '%';
            bit.style.background = colours[i % colours.length];
            bit.style.animationDelay = (Math.random() * 0.6) + 's';
            bit.style.animationDuration = (1.6 + Math.random() * 1.2) + 's';
            bit.style.transform = `rotate(${Math.random() * 360}deg)`;
            layer.appendChild(bit);
        }
    }

    window.krCourseShare = {
        setScore(score) {
            boxes.forEach(b => b.krSetScore && b.krSetScore(score));
            const el = dialog && dialog.querySelector('[data-complete-score]');
            if (el) {
                el.hidden = !score;
                el.textContent = score ? `, with ${score} on the quizzes` : '';
            }
        },
        open(options) {
            if (!dialog || typeof dialog.showModal !== 'function') return;
            this.setScore(options && options.score);
            if (!dialog.open) dialog.showModal();
            confetti();
        }
    };

    if (dialog) {
        dialog.querySelector('[data-complete-close]').addEventListener('click', () => dialog.close());
        // Click on the backdrop closes it
        dialog.addEventListener('click', (e) => { if (e.target === dialog) dialog.close(); });
    }
})();
