// Course progress and quiz scoring for lesson pages (layout: lesson).
//
// Everything is kept in the reader's browser (localStorage), one record per
// course, keyed by the course folder (e.g. /learn/micropython/):
//   { visited: { "01_intro": true, ... }, answers: { "<question id>": true|false } }
// Only the first answer to each question counts towards the score.
(function () {
    const STORAGE_PREFIX = 'kr-course:';

    const path = window.location.pathname;
    const courseKey = path.substring(0, path.lastIndexOf('/') + 1);
    const lessonId = (url) => url.split('/').pop().replace(/\.html$/, '');
    // The course's index.html is a copy of its first lesson
    let currentLesson = lessonId(path);
    if (currentLesson === '' || currentLesson === 'index') {
        const first = document.querySelector('.course-nav-item');
        if (first) currentLesson = lessonId(first.getAttribute('href') || '');
    }

    function load() {
        try {
            const data = JSON.parse(localStorage.getItem(STORAGE_PREFIX + courseKey));
            if (data && typeof data === 'object') {
                return { visited: data.visited || {}, answers: data.answers || {} };
            }
        } catch (e) { /* storage blocked or corrupt: start fresh */ }
        return { visited: {}, answers: {} };
    }

    function save(state) {
        try {
            localStorage.setItem(STORAGE_PREFIX + courseKey, JSON.stringify(state));
        } catch (e) { /* storage blocked: progress just won't persist */ }
    }

    const state = load();
    state.visited[currentLesson] = true;
    save(state);

    // --- Sidebar: progress bar, visited ticks, quiz score -------------------
    function updateSidebar() {
        const links = Array.from(document.querySelectorAll('.course-nav-item'));
        if (links.length === 0) return;

        let visited = 0;
        links.forEach(link => {
            const done = !!state.visited[lessonId(link.getAttribute('href') || '')];
            link.classList.toggle('visited', done);
            if (done) visited++;
        });

        const percent = Math.round((visited / links.length) * 100);
        const bar = document.querySelector('[data-course-progress-bar]');
        const text = document.querySelector('[data-course-progress-text]');
        if (bar) {
            bar.style.width = percent + '%';
            bar.setAttribute('aria-valuenow', percent);
        }
        if (text) {
            text.textContent = visited === links.length
                ? `Course complete! All ${links.length} lessons visited`
                : `${visited} of ${links.length} lessons visited (${percent}%)`;
        }

        const score = document.querySelector('[data-quiz-score]');
        if (score) {
            const total = Number(score.dataset.quizTotal) || 0;
            const answered = Object.values(state.answers);
            const correct = answered.filter(Boolean).length;
            score.hidden = total === 0;
            score.querySelector('[data-quiz-score-text]').textContent =
                `${correct} / ${total}` + (answered.length < total ? ` (${total - answered.length} to go)` : '');
        }
    }

    // --- Quizzes --------------------------------------------------------------
    function setupQuestion(q) {
        const qid = q.dataset.qid;
        const correctAnswers = q.dataset.answer.split(',').filter(Boolean).map(Number);
        const options = Array.from(q.querySelectorAll('.kr-quiz-option'));
        const feedback = q.querySelector('.kr-quiz-feedback');
        const explain = q.querySelector('.kr-quiz-explain');

        function reset() {
            options.forEach(o => {
                o.disabled = false;
                o.classList.remove('correct', 'wrong');
            });
            feedback.textContent = '';
            feedback.className = 'kr-quiz-feedback';
            if (explain) explain.hidden = true;
        }

        function answer(index) {
            const right = correctAnswers.includes(index);
            const firstTry = !(qid in state.answers);
            if (firstTry) {
                state.answers[qid] = right;
                save(state);
                updateSidebar();
            }

            options.forEach((o, i) => {
                o.disabled = true;
                if (correctAnswers.includes(i)) o.classList.add('correct');
                else if (i === index) o.classList.add('wrong');
            });
            if (explain) explain.hidden = false;

            feedback.className = 'kr-quiz-feedback ' + (right ? 'is-right' : 'is-wrong');
            feedback.textContent = right ? 'Correct, nice one!' : 'Not quite. The right answer is highlighted.';
            if (!firstTry) feedback.textContent += ' (Only your first try counts towards your score.)';

            const again = document.createElement('button');
            again.type = 'button';
            again.className = 'btn btn-link btn-sm p-0 ms-2 kr-quiz-again';
            again.textContent = 'Try again';
            again.addEventListener('click', reset);
            feedback.appendChild(again);
        }

        options.forEach((o, i) => o.addEventListener('click', () => answer(i)));
    }

    document.querySelectorAll('.kr-quiz-q').forEach(setupQuestion);

    // Reset button in the sidebar
    const resetLink = document.querySelector('[data-course-reset]');
    if (resetLink) {
        resetLink.addEventListener('click', (e) => {
            e.preventDefault();
            if (!confirm('Reset your progress and quiz score for this course?')) return;
            state.visited = { [currentLesson]: true };
            state.answers = {};
            save(state);
            updateSidebar();
        });
    }

    updateSidebar();
})();
