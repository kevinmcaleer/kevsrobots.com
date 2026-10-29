// The project editor's own `/` commands (issue #257).
//
// markdown-studio's editor already offers headings, tables, lists, code,
// images, links and dates under `/`. This adds a **Project** group after
// them: the canonical sections from project-sections.js, and pickers for a
// project's files, videos, parts and diagrams. project-editor.js passes the
// result to MarkdownEditor.mount({ slashCommands }).
//
// Pickers open at the cursor with a search field: type to filter, arrows
// move, Enter or Tab picks, Escape goes back to the text.
(function () {
  const ICONS = {
    file: 'M4 1.8h5l3 3v9.4H4zM9 1.8v3h3M8 7.5v4.5M6 10l2 2 2-2',
    video: 'M2 4h8.5v8H2zM10.5 7 14 5v6l-3.5-2',
    part: 'M5 5h6v6H5zM6.5 2.5V5M9.5 2.5V5M6.5 11v2.5M9.5 11v2.5M2.5 6.5H5M2.5 9.5H5M11 6.5h2.5M11 9.5h2.5',
    diagram: 'M2 2.5h5v4H2zM9 9.5h5v4H9zM4.5 6.5v5H9',
    upload: 'M8 11V3M5 6l3-3 3 3M3 11v2.5h10V11',
  };
  const MODEL_FILE_RE = /\.(stl|3mf|obj|step|stp)$/i;

  // ------------------------------------------------------------ inserting

  // Puts `md` at the cursor. A block sits between blank lines (a YouTube
  // link only becomes a player as a paragraph of its own), and typing
  // carries on below it.
  function insertAtCursor(view, md, block) {
    const state = view.state;
    const head = state.selection.main.head;
    const line = state.doc.lineAt(head);
    let lead = '';
    let tail = '';
    if (block) {
      if (head > line.from) lead = '\n\n';
      else if (line.number > 1 && state.doc.line(line.number - 1).text.trim()) lead = '\n';
      tail = md.endsWith('\n') ? '\n' : '\n\n';
    }
    const insert = lead + md + tail;
    // Focus first: when the cursor is in a table, the editor then carries
    // on in the cell the text went into.
    view.focus();
    view.dispatch({
      changes: { from: head, to: state.selection.main.to, insert },
      selection: { anchor: head + insert.length },
      scrollIntoView: true,
      userEvent: 'input',
    });
  }

  // A repository link from what was typed: a URL, or a bare
  // "github.com/you/robot" given https.
  function repoUrl(q) {
    const text = String(q || '').trim();
    if (/^https?:\/\/[^\s/]+\.[^\s]+$/i.test(text)) return text;
    if (/^[a-z0-9-]+(\.[a-z0-9-]+)+\/\S+$/i.test(text)) return 'https://' + text;
    return null;
  }

  function mdLinkText(s) {
    return String(s).replace(/([\[\]\\])/g, '\\$1');
  }

  function mdTableCell(s) {
    return String(s).replace(/\|/g, '\\|');
  }

  // ------------------------------------------------------------ picker

  // Where the cursor is on screen: in a table's cell (the `/` menu there
  // leaves the caret in the cell), or in the text.
  function caretRect(view) {
    const active = document.activeElement;
    if (active && /^T[DH]$/.test(active.tagName) && view.contentDOM.contains(active)) {
      const sel = window.getSelection();
      const rect = sel.rangeCount ? sel.getRangeAt(0).getClientRects()[0] : null;
      // A caret at the end of a cell can measure as an empty box.
      return rect && rect.height ? rect : active.getBoundingClientRect();
    }
    return view.coordsAtPos(view.state.selection.main.head);
  }

  let open = null; // the picker that's showing, so a second one replaces it

  // Opens a picker at the cursor.
  //   placeholder  the search field's hint
  //   load()       resolves to the items: { label, detail?, icon?, pick() }
  //   free(query)  optional: an item made from what was typed (a URL, say),
  //                listed first; return null when the query isn't one
  //   empty        shown when nothing matches
  function openPicker(view, opts) {
    if (open) open.close(false);

    const el = document.createElement('div');
    el.className = 'project-slash-picker';
    const listId = 'project-slash-picker-list';
    el.innerHTML =
      '<input type="text" class="form-control form-control-sm" role="combobox" aria-expanded="true" ' +
      'aria-autocomplete="list" aria-controls="' + listId + '" autocomplete="off" spellcheck="false">' +
      '<div class="project-slash-picker-list" id="' + listId + '" role="listbox"></div>';
    const input = el.querySelector('input');
    const list = el.querySelector('[role=listbox]');
    input.placeholder = opts.placeholder || 'Search…';
    input.setAttribute('aria-label', opts.placeholder || 'Search');
    document.body.appendChild(el);

    const at = caretRect(view);
    const box = view.dom.getBoundingClientRect();
    const left = at ? at.left : box.left;
    const top = at ? at.bottom + 4 : box.top;
    el.style.left = Math.max(8, Math.min(left, window.innerWidth - el.offsetWidth - 8)) + 'px';
    // Above the cursor, growing upwards, when a full list (about 320px)
    // wouldn't fit below.
    if (at && top + 320 > window.innerHeight - 8 && at.top > 320) {
      el.style.bottom = (window.innerHeight - at.top + 4) + 'px';
    } else {
      el.style.top = top + 'px';
    }

    let items = null; // null while loading
    let shown = [];
    let active = 0;

    function render() {
      const q = input.value.trim();
      const ql = q.toLowerCase();
      shown = [];
      const free = q && opts.free ? opts.free(q) : null;
      if (free) shown.push(free);
      if (items) {
        shown = shown.concat(items.filter(it => it.always ||
          !ql || (it.label + ' ' + (it.detail || '')).toLowerCase().includes(ql)));
      }
      if (active >= shown.length) active = Math.max(0, shown.length - 1);
      list.replaceChildren();
      if (!shown.length) {
        const empty = document.createElement('div');
        empty.className = 'project-slash-picker-empty';
        empty.textContent = items ? (q ? 'No matches' : opts.empty || 'Nothing here yet') : 'Loading…';
        list.appendChild(empty);
        input.removeAttribute('aria-activedescendant');
        return;
      }
      shown.forEach((it, i) => {
        const row = document.createElement('div');
        row.className = 'project-slash-picker-item' + (i === active ? ' active' : '');
        row.id = listId + '-' + i;
        row.setAttribute('role', 'option');
        row.setAttribute('aria-selected', String(i === active));
        if (it.icon) {
          row.insertAdjacentHTML('beforeend', '<svg width="16" height="16" viewBox="0 0 16 16" aria-hidden="true">' +
            '<path d="' + it.icon + '" fill="none" stroke="currentColor" stroke-width="1.3" stroke-linecap="round" stroke-linejoin="round"/></svg>');
        } else if (it.thumb) {
          const img = document.createElement('img');
          img.src = it.thumb;
          img.alt = '';
          img.loading = 'lazy';
          row.appendChild(img);
        }
        const text = document.createElement('span');
        text.className = 'project-slash-picker-text';
        const label = document.createElement('span');
        label.textContent = it.label;
        text.appendChild(label);
        if (it.detail) {
          const detail = document.createElement('small');
          detail.textContent = it.detail;
          text.appendChild(detail);
        }
        row.appendChild(text);
        row.addEventListener('mousedown', e => e.preventDefault());
        row.addEventListener('mousemove', () => { if (active !== i) { active = i; render(); } });
        row.addEventListener('click', () => choose(i));
        list.appendChild(row);
      });
      input.setAttribute('aria-activedescendant', listId + '-' + active);
      const row = list.children[active];
      if (row && row.scrollIntoView) row.scrollIntoView({ block: 'nearest' });
    }

    async function choose(i) {
      const it = shown[i];
      if (!it) return;
      close(false);
      try {
        await it.pick();
      } catch (e) {
        alert(e.message || String(e));
        view.focus();
      }
    }

    function close(refocus) {
      if (open !== picker) return;
      open = null;
      el.remove();
      document.removeEventListener('mousedown', outside, true);
      if (refocus) view.focus();
    }

    function outside(e) {
      if (!el.contains(e.target)) close(false);
    }

    input.addEventListener('input', () => { active = 0; render(); });
    input.addEventListener('keydown', e => {
      if (e.key === 'ArrowDown' || e.key === 'ArrowUp') {
        e.preventDefault();
        if (!shown.length) return;
        active = (active + (e.key === 'ArrowDown' ? 1 : -1) + shown.length) % shown.length;
        render();
      } else if (e.key === 'Enter' || (e.key === 'Tab' && !e.shiftKey && shown.length)) {
        e.preventDefault();
        choose(active);
      } else if (e.key === 'Escape') {
        e.preventDefault();
        e.stopPropagation();
        close(true);
      }
    });
    document.addEventListener('mousedown', outside, true);

    const picker = { close, el };
    open = picker;
    render();
    input.focus();
    Promise.resolve(opts.load ? opts.load() : []).then(
      loaded => { items = loaded || []; if (open === picker) render(); },
      () => { items = []; if (open === picker) render(); },
    );
    return picker;
  }

  // ------------------------------------------------------------ commands

  // The commands for MarkdownEditor's `slashCommands` option.
  //   ctx.listFiles()   resolves to the project's files: { name, url }
  //   ctx.modelFiles()  the files already loaded (sync), for 3D printed parts
  //   ctx.uploadFile()  asks for a file, uploads it, resolves to { name, url } or null
  //   ctx.listVideos()  resolves to the project's videos: { id, title }
  //   ctx.listParts()   resolves to the BOM: { name, url (catalog page or null), detail }
  //   ctx.diagrams      [{ label, md }], the diagram templates
  // The sections a document already has, worked out once per version of
  // the text (the menu asks on every keystroke while it's open).
  const presentByDoc = new WeakMap();
  function sectionsIn(state) {
    let ids = presentByDoc.get(state.doc);
    if (!ids) {
      ids = window.ProjectSections.present(state.doc.toString());
      presentByDoc.set(state.doc, ids);
    }
    return ids;
  }

  function create(ctx) {
    // A section the document already has, added with `/` or typed, is left
    // out of the menu so it isn't added twice.
    const sections = window.ProjectSections.list.filter(section => section.menu !== false).map(section => ({
      hidden: state => sectionsIn(state).has(section.id),
      id: 'section-' + section.id,
      label: section.heading,
      keywords: section.keywords.concat(['section']),
      group: 'project',
      icon: section.icon,
      block: true,
      blankBefore: true,
      insert: () => sectionTemplate(section, ctx),
    }));
    // Code repository asks for the link first. A selected placeholder
    // read as "paste here", but pasting a URL over selected text makes a
    // Markdown link of it, which the live view folds away.
    const repo = sections.find(c => c.id === 'section-code-repository');
    if (repo) {
      const heading = '# ' + window.ProjectSections.get('code-repository').heading + '\n\n';
      delete repo.insert;
      repo.run = view => openPicker(view, {
        placeholder: 'Paste your repository link…',
        empty: 'Paste a link to your code (GitHub, GitLab…)',
        free: q => {
          const url = repoUrl(q);
          return url ? {
            label: 'Add this repository',
            detail: url,
            icon: repo.icon,
            pick: () => insertAtCursor(view, heading + url + '\n', true),
          } : null;
        },
        load: async () => [],
      });
    }

    return sections.concat([
      {
        id: 'file',
        label: 'File',
        keywords: ['download', 'attachment', 'upload', 'pdf', 'zip'],
        group: 'project',
        icon: ICONS.file,
        run: view => openPicker(view, {
          placeholder: 'Find a file…',
          empty: 'No files yet: upload one',
          load: async () => {
            const files = (await ctx.listFiles()).map(f => ({
              label: f.name,
              detail: f.description || '',
              icon: ICONS.file,
              pick: () => insertAtCursor(view, '[' + mdLinkText(f.name) + '](' + f.url + ')', false),
            }));
            files.push({
              label: 'Upload a file…',
              icon: ICONS.upload,
              always: true,
              pick: async () => {
                const f = await ctx.uploadFile();
                if (f) insertAtCursor(view, '[' + mdLinkText(f.name) + '](' + f.url + ')', false);
                else view.focus();
              },
            });
            return files;
          },
        }),
      },
      {
        id: 'video',
        label: 'Video',
        keywords: ['youtube', 'embed', 'film'],
        group: 'project',
        icon: ICONS.video,
        block: true,
        run: view => openPicker(view, {
          placeholder: 'Paste a YouTube URL…',
          empty: 'Paste a YouTube link to embed it',
          free: q => {
            const id = window.ProjectEmbeds.youTubeId(q);
            return id ? {
              label: 'Embed this video',
              detail: id,
              icon: ICONS.video,
              pick: () => insertAtCursor(view, window.ProjectEmbeds.videoMarkdown(id), true),
            } : null;
          },
          load: async () => (await ctx.listVideos()).map(v => ({
            label: v.title || v.id,
            detail: 'From this project’s videos',
            thumb: 'https://img.youtube.com/vi/' + encodeURIComponent(v.id) + '/default.jpg',
            pick: () => insertAtCursor(view, window.ProjectEmbeds.videoMarkdown(v.id), true),
          })),
        }),
      },
      {
        id: 'part',
        label: 'Part',
        keywords: ['component', 'bom', 'tag'],
        group: 'project',
        icon: ICONS.part,
        run: view => openPicker(view, {
          placeholder: 'Find a part in the Bill of Materials…',
          empty: 'Add parts to the Bill of Materials first',
          load: async () => {
            // The Bill of Materials table in the text, then the structured
            // BOM rows it doesn't already name.
            const parts = bomTableParts(view.state.doc.toString());
            const byName = new Map(parts.map(p => [p.name.toLowerCase(), p]));
            for (const p of await ctx.listParts()) {
              const same = byName.get(p.name.toLowerCase());
              if (!same) parts.push(p);
              else if (!same.url) same.url = p.url; // a catalog link the table lacks
            }
            return parts.map(p => ({
              label: p.name,
              detail: p.detail || '',
              icon: ICONS.part,
              pick: () => insertAtCursor(view, p.url ? '[' + mdLinkText(p.name) + '](' + p.url + ')' : p.name, false),
            }));
          },
        }),
      },
      {
        id: 'diagram',
        label: 'Diagram',
        keywords: ['mermaid', 'flowchart', 'sequence', 'state', 'chart'],
        group: 'project',
        icon: ICONS.diagram,
        block: true,
        run: view => openPicker(view, {
          placeholder: 'Pick a diagram…',
          load: () => ctx.diagrams.map(d => ({
            label: d.label,
            detail: 'Mermaid',
            icon: ICONS.diagram,
            pick: () => insertAtCursor(view, d.md.trim() + '\n', true),
          })),
        }),
      },
    ]);
  }

  // The parts listed in the Markdown table under the Bill of Materials
  // heading: { name, url (when the cell is a link), detail }. The part is
  // the column headed Part, Item, Name or Component (else the first), and
  // Qty or Quantity gives the count.
  function bomTableParts(markdown) {
    const lines = markdown.split('\n');
    const cells = line => line.trim().replace(/^\|/, '').replace(/(?<!\\)\|$/, '')
      .split(/(?<!\\)\|/).map(c => c.trim());
    let i = lines.findIndex(line => {
      const m = line.match(/^#{1,6}\s+(.*?)\s*#*\s*$/);
      return m && window.ProjectSections.match(m[1]) === window.ProjectSections.get('bill-of-materials');
    });
    if (i < 0) return [];
    // The first table before the next heading.
    for (i++; i < lines.length && !/^\s*\|/.test(lines[i]); i++) {
      if (/^#{1,6}\s/.test(lines[i])) return [];
    }
    if (i + 1 >= lines.length || !/^\s*\|?\s*:?-{3,}/.test(lines[i + 1])) return [];
    const header = cells(lines[i]).map(h => h.toLowerCase());
    let nameCol = header.findIndex(h => /^(part|parts|item|name|component)s?$/.test(h));
    if (nameCol < 0) nameCol = 0;
    const qtyCol = header.findIndex(h => /^(qty|quantity|count)$/.test(h));
    const parts = [];
    for (let r = i + 2; r < lines.length && /^\s*\|/.test(lines[r]); r++) {
      const row = cells(lines[r]);
      const raw = (row[nameCol] || '').replace(/\\\|/g, '|');
      const link = raw.match(/^\[([^\]]+)\]\(([^)\s]+)[^)]*\)$/);
      const name = (link ? link[1] : raw).replace(/[*_`]/g, '').trim();
      if (!name) continue;
      const qty = qtyCol >= 0 ? row[qtyCol] : '';
      parts.push({
        name,
        url: link ? link[2] : null,
        detail: (qty ? qty + ' \u00d7 \u00b7 ' : '') + 'in the Bill of Materials table',
      });
    }
    return parts;
  }

  // What a canonical section command inserts: its H1 (exactly as
  // project-sections.js spells it) and a start on its contents, with the
  // first thing to fill in selected.
  function sectionTemplate(section, ctx) {
    const h = '# ' + section.heading + '\n\n';
    switch (section.id) {
      case 'bill-of-materials':
        return { before: h + '| Part | Qty | Notes |\n| --- | --- | --- |\n| ', placeholder: 'Part name', after: ' | 1 |  |\n' };
      case 'assembly-instructions':
        return { before: h + '1. ', placeholder: 'First step', after: '\n' };
      case 'wiring':
        return { before: h + '| From | To | Notes |\n| --- | --- | --- |\n| ', placeholder: 'Pico GP0', after: ' |  |  |\n' };
      case '3d-printed-parts': {
        const models = (ctx.modelFiles ? ctx.modelFiles() : []).filter(f => MODEL_FILE_RE.test(f.name));
        const head = h + '| Part | Qty | Notes |\n| --- | --- | --- |\n';
        if (!models.length) return { before: head + '| ', placeholder: 'Part name', after: ' | 1 |  |\n' };
        const rows = models.map(f => '| [' + mdTableCell(mdLinkText(f.name)) + '](' + f.url + ') | 1 |  |\n').join('');
        return { before: head + rows };
      }
      default:
        return { before: h };
    }
  }

  window.ProjectSlashCommands = { create, openPicker, insertAtCursor, bomTableParts };
})();
