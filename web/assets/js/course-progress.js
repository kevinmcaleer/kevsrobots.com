// Course progress and quiz scoring for lesson pages (layout: lesson).
//
// Everything is kept in the reader's browser (localStorage), one record per
// course, keyed by the course folder (e.g. /learn/micropython/):
//   { visited: { "01_intro": true, ... }, answers: { "<question id>": true|false },
//     last: "01_intro", updated: <ms since 1970>,
//     completed: <ms since 1970, set when the "Course complete" dialog was shown> }
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
                return { ...data, visited: data.visited || {}, answers: data.answers || {} };
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
    // For "Continue learning" on /learn/ (learn-progress.js): where the
    // reader was last, and when.
    state.last = currentLesson;
    state.updated = Date.now();
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

        const shareLink = document.querySelector('[data-course-share-open]');
        if (shareLink) shareLink.hidden = visited !== links.length;

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

    // --- Course complete: congratulate once, then offer to share ------------
    // course-share.js (loaded only for courses with social cards) provides
    // window.krCourseShare. state.completed remembers we've said well done.
    function quizScore() {
        const el = document.querySelector('[data-quiz-score]');
        const total = el ? Number(el.dataset.quizTotal) || 0 : 0;
        const answered = Object.values(state.answers);
        if (total === 0 || answered.length < total) return '';
        return `${answered.filter(Boolean).length}/${total}`;
    }

    function isComplete() {
        const links = Array.from(document.querySelectorAll('.course-nav-item'));
        return links.length > 0 &&
            links.every(link => state.visited[lessonId(link.getAttribute('href') || '')]);
    }

    function checkComplete() {
        if (!window.krCourseShare) return;
        window.krCourseShare.setScore(quizScore());
        if (isComplete() && !state.completed) {
            state.completed = Date.now();
            save(state);
            window.krCourseShare.open({ score: quizScore() });
        }
    }

    const shareOpen = document.querySelector('[data-course-share-open]');
    if (shareOpen) {
        shareOpen.addEventListener('click', (e) => {
            e.preventDefault();
            if (window.krCourseShare) window.krCourseShare.open({ score: quizScore() });
        });
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
                if (window.krCourseShare) window.krCourseShare.setScore(quizScore());
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
            delete state.completed;
            save(state);
            updateSidebar();
        });
    }

    updateSidebar();
    checkComplete();
})();
