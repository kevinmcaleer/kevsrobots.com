// Course overview (#635, _layouts/course_overview.html): ticks the lessons
// the reader has opened and turns "Start the course" into "Resume", from the
// course's browser record (course-progress.js). For signed-in readers,
// progress-sync.js first brings in what they did on other devices.
(function () {
    const root = document.querySelector('[data-course-overview]');
    if (!root) return;
    const key = root.dataset.courseKey;
    const lessons = Number(root.dataset.lessons) || 0;

    function render() {
        let data = null;
        try { data = JSON.parse(localStorage.getItem('kr-course:' + key)); } catch (e) { /* none */ }
        const visited = (data && data.visited) || {};
        let count = 0;
        root.querySelectorAll('[data-lesson-id]').forEach(li => {
            const done = !!visited[li.dataset.lessonId];
            if (done) count++;
            li.classList.toggle('is-done', done);
            li.querySelector('.co-lesson-done').hidden = !done;
        });
        if (count === 0) return;

        const finished = count >= lessons || !!(data && data.completed);
        const percent = finished ? 100 : Math.round((count / lessons) * 100);
        root.querySelector('[data-co-progress]').hidden = false;
        root.querySelector('[data-co-bar]').style.width = percent + '%';
        root.querySelector('[data-co-progress-text]').textContent = finished
            ? 'You finished this course'
            : `${count} of ${lessons} lessons done`;
        const go = root.querySelector('[data-co-go]');
        if (finished) {
            go.textContent = 'Review the course';
        } else {
            go.textContent = 'Resume';
            if (data.last) go.href = data.last + '.html';
        }
    }

    render();
    if (window.krProgressSync && window.krProgressSync.signedIn()) {
        window.krProgressSync.syncAll().then(body => { if (body) render(); });
    }
})();
