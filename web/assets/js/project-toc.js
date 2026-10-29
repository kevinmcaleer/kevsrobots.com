// "On this page" navigation for projects (issue #259).
//
// Shared by the project page (view.html) and the editor (editor.html) so
// both build the same list, in the article layout's `kr-toc` look:
//
//   - the five canonical sections (project-sections.js) come first, in a
//     highlighted group with icons, in their fixed order, wherever the
//     author put them in the document;
//   - then the remaining headings: the top level (H1, or H2 when a
//     project has no H1s) with the next level nested beneath;
//   - then extras, the sections the page renders outside the Markdown.
//
// ProjectToc.render(nav, entries, options)
//   entries  [{ level, text, key }] in document order. `key` is what the
//            caller gets back on a click: an element id on the view page,
//            a line number in the editor.
//   options  { extras: [{ text, key, section? }], onSelect(key, entry),
//              empty: 'text when there are no headings' }
//            An extra with `section` (a canonical id) joins the canonical
//            group unless a heading already names that section.
// Returns { links: Map(key -> <a>), setActive(key) }.
(function () {
  function esc(t) {
    const d = document.createElement('div');
    d.textContent = t == null ? '' : String(t);
    return d.innerHTML;
  }

  // Group the entries; exported for tests and for callers that need the
  // shape (view.html uses it to place the BOM under its heading).
  function group(entries, extras) {
    const sections = window.ProjectSections;
    const levels = entries.map(e => e.level);
    const top = levels.length ? Math.min.apply(null, levels) : 1;

    const canonical = new Map(); // section id -> node
    const tree = [];
    let parent = null;
    entries.forEach(e => {
      if (e.level > top + 1) return;
      const section = sections ? sections.match(e.text) : null;
      const node = { entry: e, section: null, children: [] };
      if (section && !canonical.has(section.id)) {
        node.section = section;
        canonical.set(section.id, node);
        // A canonical H1 keeps its subheadings; a canonical H2 is lifted
        // out on its own.
        parent = e.level === top ? node : parent;
        return;
      }
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
      if (section) {
        if (!canonical.has(section.id)) {
          canonical.set(section.id, { entry: ex, section, children: [], extra: true });
        }
        return;
      }
      trailing.push({ entry: ex, section: null, children: [], extra: true });
    });

    const ordered = sections
      ? sections.list.filter(s => canonical.has(s.id)).map(s => canonical.get(s.id))
      : [];
    return { canonical: ordered, tree, extras: trailing };
  }

  function render(nav, entries, options) {
    const opts = options || {};
    const g = group(entries || [], opts.extras);
    const links = new Map();

    function link(node, cls) {
      const e = node.entry;
      const icon = node.section ? window.ProjectSections.iconSvg(node.section) : '';
      // Canonical entries read the same on every project.
      const text = node.section ? node.section.heading : e.text;
      return '<a href="' + (typeof e.key === 'string' ? '#' + esc(e.key) : '#') + '"' +
        (cls ? ' class="' + cls + '"' : '') + ' data-toc-key="' + esc(e.key) + '">' +
        icon + '<span>' + esc(text) + '</span></a>';
    }
    function items(nodes, cls) {
      return nodes.map(n => '<li>' + link(n, cls) +
        (n.children.length ? '<ol>' + items(n.children, 'kr-toc-sub') + '</ol>' : '') +
        '</li>').join('');
    }

    let html = '<p>On this page</p>';
    if (g.canonical.length) html += '<ol class="kr-toc-sections">' + items(g.canonical, 'kr-toc-section') + '</ol>';
    if (g.tree.length) html += '<ol>' + items(g.tree) + '</ol>';
    if (!g.canonical.length && !g.tree.length && opts.empty) html += '<div class="kr-toc-empty">' + esc(opts.empty) + '</div>';
    if (g.extras.length) html += '<ol class="kr-toc-extras">' + items(g.extras) + '</ol>';
    nav.innerHTML = html;

    const all = [].concat(g.canonical, g.tree, g.extras);
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
