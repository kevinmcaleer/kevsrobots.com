// Tabs for the homepage "Kev's builds" band (_includes/home/builds.html).
// Every tab's cards are already in the page; this only shows one panel at a
// time. Arrow keys move between tabs, as in any tab list.
(function () {
    const band = document.getElementById('kevs-builds');
    if (!band) return;
    const list = band.querySelector('.kb-tabs');
    const tabs = Array.from(list.querySelectorAll('[role="tab"]'));

    function select(tab, focus) {
        tabs.forEach(t => {
            const on = t === tab;
            t.setAttribute('aria-selected', String(on));
            t.tabIndex = on ? 0 : -1;
            document.getElementById(t.getAttribute('aria-controls')).hidden = !on;
        });
        if (focus) tab.focus();
    }

    tabs.forEach((tab, i) => {
        tab.addEventListener('click', () => select(tab, false));
        tab.addEventListener('keydown', e => {
            let next = null;
            if (e.key === 'ArrowRight') next = tabs[(i + 1) % tabs.length];
            if (e.key === 'ArrowLeft') next = tabs[(i - 1 + tabs.length) % tabs.length];
            if (e.key === 'Home') next = tabs[0];
            if (e.key === 'End') next = tabs[tabs.length - 1];
            if (next) { e.preventDefault(); select(next, true); }
        });
    });
    list.hidden = false;
})();
