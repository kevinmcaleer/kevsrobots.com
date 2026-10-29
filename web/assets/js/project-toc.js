// "On this page" navigation for projects (issue #259).
//
// Shared by the project page (view.html) and the editor (editor.html) so
// both build the same list, in the article layout's `kr-toc` look:
//
//   - the headings in the order they appear in the document: the top
//     level (H1, or H2 when a project has no H1s) with the next level
//     nested beneath. The canonical sections (project-sections.js) keep
//     their place but stand out, with an icon;
//   - then extras, the sections the page renders outside the Markdown.
//
// ProjectToc.render(nav, entries, options)
//   entries  [{ level, text, key }] in document order. `key` is what the
//            caller gets back on a click: an element id on the view page,
//            a line number in the editor.
//   options  { extras: [{ text, key, section? }], onSelect(key, entry),
//              empty: 'text when there are no headings' }
//            An extra with `section` (a canonical id) gets that section's
//            icon, and is left out when a heading already names it.
// Returns { links: Map(key -> <a>), setActive(key) }.
(function () {
  function esc(t) {
    const d = document.createElement('div');
    d.textContent = t == null ? '' : String(t);
    return d.innerHTML;
  }

  // Nest the entries under the top level, in document order.
  function group(entries, extras) {
    const sections = window.ProjectSections;
    const levels = entries.map(e => e.level);
    const top = levels.length ? Math.min.apply(null, levels) : 1;

    const named = new Set(); // canonical ids a heading names
    const tree = [];
    let parent = null;
    entries.forEach(e => {
      if (e.level > top + 1) return;
      const section = sections ? sections.match(e.text) : null;
      if (section) named.add(section.id);
      const node = { entry: e, section, children: [] };
      if (e.level === top) {
        tree.push(node);
        parent = node;
      } else if (parent) {
        parent.children.push(node);
      } else {
        tree.push(node);
      }
    });

    const trailing = [];
    (extras || []).forEach(ex => {
      const section = ex.section && sections ? sections.get(ex.section) : null;
      if (section && named.has(section.id)) return;
      trailing.push({ entry: ex, section, children: [], extra: true });
    });
    return { tree, extras: trailing };
  }

  function render(nav, entries, options) {
    const opts = options || {};
    const g = group(entries || [], opts.extras);
    const links = new Map();

    function link(node, sub) {
      const e = node.entry;
      const cls = [sub ? 'kr-toc-sub' : '', node.section ? 'kr-toc-section' : ''].filter(Boolean).join(' ');
      const icon = node.section ? window.ProjectSections.iconSvg(node.section) : '';
      return '<a href="' + (typeof e.key === 'string' ? '#' + esc(e.key) : '#') + '"' +
        (cls ? ' class="' + cls + '"' : '') + ' data-toc-key="' + esc(e.key) + '">' +
        icon + '<span>' + esc(e.text) + '</span></a>';
    }
    function items(nodes, sub) {
      return nodes.map(n => '<li>' + link(n, sub) +
        (n.children.length ? '<ol>' + items(n.children, true) + '</ol>' : '') +
        '</li>').join('');
    }

    let html = '<p>On this page</p>';
    if (g.tree.length) html += '<ol>' + items(g.tree) + '</ol>';
    else if (opts.empty) html += '<div class="kr-toc-empty">' + esc(opts.empty) + '</div>';
    if (g.extras.length) html += '<ol class="kr-toc-extras">' + items(g.extras) + '</ol>';
    nav.innerHTML = html;

    const all = [].concat(g.tree, g.extras);
    const byKey = new Map();
    (function index(nodes) {
      nodes.forEach(n => { byKey.set(String(n.entry.key), n.entry); index(n.children); });
    })(all);

    let active = null;
    function setActive(key) {
      const a = links.get(String(key));
      if (active) active.classList.remove('is-active');
      active = a || null;
      if (active) active.classList.add('is-active');
    }

    nav.querySelectorAll('a[data-toc-key]').forEach(a => {
      const key = a.dataset.tocKey;
      links.set(key, a);
      a.addEventListener('click', ev => {
        ev.preventDefault();
        setActive(key);
        if (opts.onSelect) opts.onSelect(byKey.get(key).key, byKey.get(key));
      });
    });

    return { links, setActive };
  }

  window.ProjectToc = { group, render };
})();
