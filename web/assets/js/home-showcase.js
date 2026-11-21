/* Homepage "Made by the community" showcase (_includes/home/showcase.html).
 *
 * Tabs: Staff picks, Made with Snakie, Made with BetterCAD and, when signed
 * in, From people you follow. Nothing is fetched until the section is near
 * the screen. Each tab is fetched once, on first view. Cards are drawn by
 * kr-project-card.js. If the API can't be reached on first load the whole
 * section hides, like the old Featured Projects widget did.
 */
(function () {
  'use strict';

  var API = window.KR_PROJECTS_API || 'https://projects.kevsrobots.com';
  var LIMIT = 6;
  var section = document.getElementById('home-showcase');
  if (!section || !window.KrProjectCard) return;

  var grid = document.getElementById('kr-showcase-grid');
  var empty = section.querySelector('.kr-showcase-empty');
  var tabs = Array.prototype.slice.call(section.querySelectorAll('[role="tab"]'));
  var cache = {};
  var started = false;
  var current = 'staff';

  var SOURCES = {
    staff: { url: '/api/projects/featured?limit=' + LIMIT, empty: 'No staff picks yet.' },
    snakie: { url: '/api/projects?made_with=snakie&limit=' + LIMIT, empty: 'No published projects made with Snakie yet. Send your wiring from Snakie to a project to be the first.' },
    bettercad: { url: '/api/projects?made_with=bettercad&limit=' + LIMIT, empty: 'No published projects made with BetterCAD yet. Send build steps from BetterCAD to a project to be the first.' },
    following: { url: '/api/projects?following=true&limit=' + LIMIT, empty: 'Nothing new from people you follow. Follow makers from their project cards.' }
  };

  if (window.KrProjectCard.viewer()) {
    section.querySelector('[data-tab="following"]').hidden = false;
  }

  function load(tab) {
    if (!cache[tab]) {
      var fetcher = window.ProjectAuth ? window.ProjectAuth.apiFetch : function (u) { return fetch(u, { credentials: 'include' }); };
      cache[tab] = fetcher(API + SOURCES[tab].url).then(function (r) {
        if (!r.ok) throw new Error('showcase ' + r.status);
        return r.json();
      });
      cache[tab].catch(function () { delete cache[tab]; });
    }
    return cache[tab];
  }

  function select(tab, focus) {
    current = tab;
    tabs.forEach(function (t) {
      var on = t.getAttribute('data-tab') === tab;
      t.setAttribute('aria-selected', on ? 'true' : 'false');
      t.tabIndex = on ? 0 : -1;
      if (on) {
        grid.setAttribute('aria-labelledby', t.id);
        if (focus) t.focus();
      }
    });
    grid.setAttribute('aria-busy', 'true');
    return load(tab).then(function (projects) {
      if (current !== tab) return;
      grid.removeAttribute('aria-busy');
      empty.hidden = projects.length > 0;
      empty.textContent = SOURCES[tab].empty;
      window.KrProjectCard.render(grid, projects);
    }, function () {
      if (current !== tab) return;
      grid.removeAttribute('aria-busy');
      grid.innerHTML = '';
      empty.hidden = false;
      empty.textContent = 'Projects could not be loaded just now. Try again in a minute.';
    });
  }

  function start() {
    if (started) return;
    started = true;
    select(current);
    // The API is down: hide the section rather than show an empty box.
    load(current).catch(function () { section.hidden = true; });
  }

  section.addEventListener('click', function (e) {
    var t = e.target.closest('[role="tab"]');
    if (t) { start(); select(t.getAttribute('data-tab')); }
  });

  section.addEventListener('keydown', function (e) {
    var t = e.target.closest('[role="tab"]');
    if (!t || (e.key !== 'ArrowRight' && e.key !== 'ArrowLeft')) return;
    var shown = tabs.filter(function (x) { return !x.hidden; });
    var i = shown.indexOf(t) + (e.key === 'ArrowRight' ? 1 : -1);
    var next = shown[(i + shown.length) % shown.length];
    select(next.getAttribute('data-tab'), true);
    e.preventDefault();
  });

  // "See projects" buttons in the suite band pick a tab.
  document.addEventListener('click', function (e) {
    var link = e.target.closest('[data-showcase-tab]');
    if (!link) return;
    current = link.getAttribute('data-showcase-tab');
    if (started) select(current); else start();
  });

  if ('IntersectionObserver' in window) {
    var io = new IntersectionObserver(function (entries) {
      if (entries.some(function (en) { return en.isIntersecting; })) { io.disconnect(); start(); }
    }, { rootMargin: '400px 0px' });
    io.observe(section);
  } else {
    start();
  }
})();
