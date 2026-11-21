// Lesson sidebar behaviour for layout: lesson.
//
// Wide screens: the sidebar sticks beside the lesson with its own scroll, and
// once you scroll past the lesson title it tightens to a slim progress card and
// a compact list of titles + ticks. Scroll back to the top and it opens again.
//
// Phones: the lesson list folds behind "Show all N lessons", and once the
// course card scrolls away a slim bar sticks under the top bar. Tapping it
// opens the full list as a sheet.
(function () {
    const nav = document.querySelector('[data-course-nav]');
    if (!nav) return;

    const wide = window.matchMedia('(min-width: 992px)');
    const lessonsEl = nav.querySelector('[data-course-lessons]');
    const toggle = nav.querySelector('[data-course-nav-toggle]');
    const title = document.querySelector('[data-lesson-title]');
    const links = () => Array.from(nav.querySelectorAll('.course-nav-item'));
    const total = links().length;
    const currentIndex = Math.max(0, links().findIndex(a => a.classList.contains('active')));
    const current = links()[currentIndex];
    const courseName = (nav.querySelector('.course-nav-title') || {}).textContent || '';

    // --- Height of the site's sticky top bar --------------------------------
    function stickyTop() {
        let h = 0;
        document.querySelectorAll('.sticky-top').forEach(el => {
            if (el.closest('[data-course-nav]') || el.offsetParent === null) return;
            if (getComputedStyle(el).position !== 'sticky') return;
            h = Math.max(h, el.offsetHeight);
        });
        return h;
    }
    function setStickyTop() {
        document.documentElement.style.setProperty('--kr-sticky-top', stickyTop() + 'px');
    }

    // Scrolls a list so `item` sits comfortably in view, without moving the page
    function revealIn(box, item, smooth) {
        if (!box || !item) return;
        const b = box.getBoundingClientRect();
        const r = item.getBoundingClientRect();
        if (r.top >= b.top + 40 && r.bottom <= b.bottom - 40) return;
        const to = box.scrollTop + (r.top - b.top) - box.clientHeight / 3;
        box.scrollTo({ top: Math.max(0, to), behavior: smooth ? 'smooth' : 'auto' });
    }

    // --- Wide screens: collapse on scroll ------------------------------------
    // A little slack either side of the title stops it flickering at the edge.
    const SLACK = 60;
    function titleBottom() {
        if (!title) return 200;
        return title.getBoundingClientRect().bottom + window.scrollY - stickyTop();
    }
    function updateCompact() {
        if (!wide.matches) {
            nav.classList.remove('is-compact');
            return;
        }
        const y = window.scrollY;
        const edge = titleBottom();
        const compact = nav.classList.contains('is-compact');
        if (!compact && y > edge + SLACK) {
            nav.classList.add('is-compact');
            window.setTimeout(() => revealIn(nav, current, true), 220);
        } else if (compact && y < edge - SLACK) {
            nav.classList.remove('is-compact');
            // Back at the top: show the full course card again
            nav.scrollTo({ top: 0, behavior: 'smooth' });
        }
    }

    // --- Phones: fold the list, slim bar, lesson sheet -----------------------
    if (toggle) {
        const label = () => {
            const open = nav.classList.contains('is-open');
            toggle.textContent = open ? 'Hide lessons' : `Show all ${total} lessons`;
            toggle.setAttribute('aria-expanded', open ? 'true' : 'false');
        };
        toggle.addEventListener('click', () => {
            nav.classList.toggle('is-open');
            label();
        });
        label();
    }

    const bar = document.createElement('button');
    bar.type = 'button';
    bar.className = 'course-nav-bar';
    bar.setAttribute('aria-haspopup', 'dialog');
    bar.innerHTML =
        '<span class="course-nav-bar-row">' +
        '<b class="course-nav-bar-count"></b>' +
        '<span class="course-nav-bar-title"></span>' +
        '<span class="course-nav-bar-open">Lessons <i class="fa-solid fa-chevron-down"></i></span>' +
        '</span><span class="course-nav-bar-progress"><i></i></span>';
    bar.querySelector('.course-nav-bar-count').textContent = `${currentIndex + 1} of ${total}`;
    bar.querySelector('.course-nav-bar-title').textContent = current ? current.textContent.trim() : courseName;
    bar.setAttribute('aria-label', `Lesson ${currentIndex + 1} of ${total}. Show all lessons`);
    document.body.appendChild(bar);

    const sheet = document.createElement('div');
    sheet.className = 'course-sheet';
    sheet.hidden = true;
    sheet.innerHTML =
        '<div class="course-sheet-scrim" data-sheet-close></div>' +
        '<div class="course-sheet-panel" role="dialog" aria-modal="true" aria-label="Lessons in this course">' +
        '<div class="course-sheet-grab" aria-hidden="true"></div>' +
        '<div class="course-sheet-head"><b class="course-sheet-title"></b>' +
        '<span class="course-sheet-meta"><span class="course-sheet-percent"></span> · ' +
        '<a href="#" data-course-reset>Reset progress</a></span>' +
        '<button type="button" class="course-sheet-close" data-sheet-close aria-label="Close lessons">&times;</button></div>' +
        '<div class="course-sheet-body"></div></div>';
    sheet.querySelector('.course-sheet-title').textContent = courseName;
    document.body.appendChild(sheet);
    const sheetBody = sheet.querySelector('.course-sheet-body');

    let percent = Number((nav.querySelector('[data-course-progress-bar]') || {}).getAttribute?.('aria-valuenow')) || 0;
    function showPercent() {
        bar.querySelector('.course-nav-bar-progress i').style.width = percent + '%';
        sheet.querySelector('.course-sheet-percent').textContent = `${percent}% complete`;
    }
    document.addEventListener('kr-course-progress', (e) => {
        percent = e.detail.percent;
        showPercent();
        if (!sheet.hidden) fillSheet();
    });
    showPercent();

    function fillSheet() {
        // A fresh copy each time, so ticks and counts are current
        const copy = lessonsEl.cloneNode(true);
        copy.removeAttribute('id');
        copy.classList.add('is-compact-list');
        sheetBody.replaceChildren(copy);
    }
    let lastFocus = null;
    function openSheet() {
        fillSheet();
        lastFocus = document.activeElement;
        sheet.hidden = false;
        document.documentElement.classList.add('course-sheet-open');
        requestAnimationFrame(() => {
            sheet.classList.add('is-open');
            revealIn(sheetBody, sheetBody.querySelector('.course-nav-item.active'), false);
            sheet.querySelector('.course-sheet-close').focus({ preventScroll: true });
        });
    }
    function closeSheet() {
        if (sheet.hidden) return;
        sheet.classList.remove('is-open');
        document.documentElement.classList.remove('course-sheet-open');
        window.setTimeout(() => { sheet.hidden = true; }, 200);
        if (lastFocus) lastFocus.focus({ preventScroll: true });
    }
    bar.addEventListener('click', openSheet);
    sheet.addEventListener('click', (e) => {
        if (e.target.closest('[data-sheet-close]')) closeSheet();
    });
    document.addEventListener('keydown', (e) => {
        if (e.key === 'Escape') closeSheet();
    });

    function updateBar() {
        if (wide.matches) {
            bar.classList.remove('is-shown');
            closeSheet();
            return;
        }
        // Show the bar once the course card (and its toggle) has scrolled away
        const top = nav.querySelector('.course-nav-top');
        const gone = top && top.getBoundingClientRect().bottom < stickyTop();
        bar.classList.toggle('is-shown', !!gone);
    }

    // --- Wiring ---------------------------------------------------------------
    let ticking = false;
    function onScroll() {
        if (ticking) return;
        ticking = true;
        requestAnimationFrame(() => {
            ticking = false;
            updateCompact();
            updateBar();
        });
    }
    function onResize() {
        setStickyTop();
        onScroll();
    }

    setStickyTop();
    // Opened part way down the page: start compact, current lesson in view
    updateCompact();
    if (nav.classList.contains('is-compact')) revealIn(nav, current, false);
    updateBar();
    // Turn the animations on only after the first layout, so the page doesn't
    // animate into place when it opens part way down
    requestAnimationFrame(() => requestAnimationFrame(() => nav.classList.add('is-ready')));

    window.addEventListener('scroll', onScroll, { passive: true });
    window.addEventListener('resize', onResize);
    if (wide.addEventListener) wide.addEventListener('change', onResize);
})();
