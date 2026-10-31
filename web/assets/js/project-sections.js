// Canonical project sections (issue #257).
//
// The one list of section names a project page can have. The editor's `/`
// commands insert these headings (except any marked `menu: false`), and the page navigation (#259) and the
// view page recognise them, so the spelling is the same everywhere. Add a
// section here, not in the places that use it.
(function () {
  const SECTIONS = [
    {
      id: 'bill-of-materials',
      heading: 'Bill of Materials',
      // Other headings that count as this section, in any case.
      aliases: ['BOM', 'Parts', 'Parts list'],
      keywords: ['bom', 'parts list', 'components', 'shopping list'],
      icon: 'M5.5 2.5h5v2h-5zM5.5 3.5H3.5v10h9v-10h-2M5.5 7.5h5M5.5 10h5',
    },
    {
      id: 'assembly-instructions',
      heading: 'Assembly instructions',
      keywords: ['assembly', 'build', 'steps', 'instructions', 'how to'],
      icon: 'M2.5 3.5h1v3M2.5 6.5h2M6.5 4h7M6.5 8h7M6.5 12h7M2.5 10.5h2l-2 2.5h2',
    },
    {
      id: 'wiring',
      heading: 'Wiring',
      keywords: ['circuit', 'electronics', 'connections', 'schematic', 'pins'],
      icon: 'M2 4.5h3.5a2 2 0 0 1 2 2v3a2 2 0 0 0 2 2H14M2 11.5h2.5M11.5 4.5H14',
    },
    {
      id: 'code',
      heading: 'Code',
      keywords: ['program', 'software', 'firmware', 'sketch', 'micropython'],
      icon: 'M5.5 4.5 2 8l3.5 3.5M10.5 4.5 14 8l-3.5 3.5',
      // Still recognised in the page navigation and on cards, but not
      // offered in the `/` menu: it read as a twin of the editor's own
      // "Code block", and "Code repository" is the section to add.
      menu: false,
    },
    {
      id: 'code-repository',
      heading: 'Code repository',
      keywords: ['repo', 'repository', 'github', 'git', 'source'],
      icon: 'M4.5 2.5v11M11.5 6.5a1.5 1.5 0 1 0 0-3 1.5 1.5 0 0 0 0 3zM11.5 6.5c0 3.5-7 2-7 5.5',
    },
    {
      id: '3d-printed-parts',
      heading: '3D printed parts',
      keywords: ['3d', 'printed', 'print', 'stl', 'models', 'cad'],
      icon: 'M8 1.8 13.5 5v6L8 14.2 2.5 11V5zM2.5 5 8 8.2 13.5 5M8 8.2v6',
    },
    {
      id: 'videos',
      heading: 'Videos',
      keywords: ['youtube', 'video', 'film', 'clips'],
      icon: 'M2 4h8.5v8H2zM10.5 7 14 5v6l-3.5-2',
    },
    {
      // A `::: gallery <layout>` block of the project's pictures under it
      // (ProjectGallery, project-gallery.js).
      id: 'gallery',
      heading: 'Gallery',
      aliases: ['Photos', 'Pictures'],
      keywords: ['photos', 'pictures', 'images', 'filmstrip', 'bento', 'lightbox', 'slideshow'],
      icon: 'M2 3.5h12v9H2zM2 10l3.5-3 3 2.5 2-1.5 3.5 3M10.5 6.5h.01',
    },
  ];

  // "Bill of Materials", "bill of materials " and "Bill-of-materials" are
  // the same section.
  function normalise(text) {
    return String(text || '').toLowerCase().replace(/[^a-z0-9]+/g, ' ').trim();
  }

  const byName = new Map(SECTIONS.flatMap(s =>
    [s.heading].concat(s.aliases || []).map(name => [normalise(name), s])));

  // The section's line icon as inline SVG (issue #259).
  function iconSvg(section, size) {
    const px = size || 16;
    return '<svg class="project-section-icon" width="' + px + '" height="' + px + '" viewBox="0 0 16 16" aria-hidden="true">' +
      '<path d="' + section.icon + '" fill="none" stroke="currentColor" stroke-width="1.3" stroke-linecap="round" stroke-linejoin="round"/></svg>';
  }

  // Small icons for the canonical sections a project has, for the hub and
  // search cards. `ids` comes from the list API's `sections` field.
  function badgesHtml(ids) {
    const have = new Set(ids || []);
    const items = SECTIONS.filter(s => have.has(s.id));
    if (!items.length) return '';
    const names = items.map(s => s.heading).join(', ');
    return '<span class="project-section-badges" role="img" title="Has: ' + names + '" aria-label="Has: ' + names + '">' +
      items.map(s => iconSvg(s, 14)).join('') + '</span>';
  }

  // The lines inside closed `::: cards` blocks (project-cards.js): a card's
  // title is a label, not a section of the page.
  function cardLines(lines) {
    const skip = new Set();
    for (let i = 0; i < lines.length; i++) {
      if (!/^ {0,3}:::[ \t]*cards\b/i.test(lines[i])) continue;
      for (let j = i + 1; j < lines.length; j++) {
        if (/^ {0,3}:::[ \t]*cards\b/i.test(lines[j])) break;
        if (/^ {0,3}:::[ \t]*$/.test(lines[j])) {
          for (let k = i; k <= j; k++) skip.add(k);
          i = j;
          break;
        }
      }
    }
    return skip;
  }

  // The Markdown's headings, outside code fences and cards: { level, text, line }
  // with `line` counted from 0.
  function headings(markdown) {
    const out = [];
    let fence = null;
    const lines = String(markdown || '').split('\n');
    const cards = cardLines(lines);
    lines.forEach((line, i) => {
      const f = line.match(/^ {0,3}(`{3,}|~{3,})/);
      if (f) {
        if (fence === null) fence = f[1];
        else if (f[1][0] === fence[0] && f[1].length >= fence.length) fence = null;
        return;
      }
      if (fence !== null || cards.has(i)) return;
      const m = line.match(/^ {0,3}(#{1,6})\s+(.+?)\s*#*\s*$/);
      if (m) out.push({ level: m[1].length, text: m[2].trim(), line: i });
    });
    return out;
  }

  // The first table in a section, before the next heading:
  //   { header: ['part', 'qty', ...] (lower case), rows: [[cell, ...]],
  //     headerLine, lines: [line numbers of the rows] }
  // or null. Cells are trimmed; escaped pipes stay escaped. When several
  // headings name the section ("Bill of Materials" and a later "BOM"),
  // the first of them with a table counts.
  function sectionTable(markdown, id) {
    const lines = String(markdown || '').split('\n');
    const cells = line => line.trim().replace(/^\|/, '').replace(/(?<!\\)\|$/, '')
      .split(/(?<!\\)\|/).map(c => c.trim());
    const tableAfter = line => {
      let i = line + 1;
      for (; i < lines.length && !/^\s*\|/.test(lines[i]); i++) {
        if (/^ {0,3}#{1,6}\s/.test(lines[i])) return null;
      }
      if (i + 1 >= lines.length || !/^\s*\|?\s*:?-{3,}/.test(lines[i + 1])) return null;
      const table = { header: cells(lines[i]).map(h => h.toLowerCase()), rows: [], headerLine: i, lines: [] };
      for (let r = i + 2; r < lines.length && /^\s*\|/.test(lines[r]); r++) {
        table.rows.push(cells(lines[r]));
        table.lines.push(r);
      }
      return table;
    };
    for (const h of headings(markdown)) {
      const section = byName.get(normalise(h.text));
      if (!section || section.id !== id) continue;
      const table = tableAfter(h.line);
      if (table) return table;
    }
    return null;
  }

  // The text under a section's heading, up to the next heading at its
  // level or above, or null when there's no such heading.
  function sectionText(markdown, id) {
    const all = headings(markdown);
    const at = all.findIndex(h => {
      const section = byName.get(normalise(h.text));
      return section && section.id === id;
    });
    if (at < 0) return null;
    const end = all.slice(at + 1).find(h => h.level <= all[at].level);
    const lines = String(markdown || '').split('\n');
    return lines.slice(all[at].line + 1, end ? end.line : lines.length).join('\n').trim();
  }

  // Ids of the canonical sections the Markdown has a heading for.
  function present(markdown) {
    const ids = new Set();
    headings(markdown).forEach(h => {
      const section = byName.get(normalise(h.text));
      if (section) ids.add(section.id);
    });
    return ids;
  }

  window.ProjectSections = {
    list: SECTIONS,
    get: id => SECTIONS.find(s => s.id === id) || null,
    // The canonical section a heading's text names, or null.
    match: text => byName.get(normalise(text)) || null,
    iconSvg,
    badgesHtml,
    headings,
    present,
    sectionTable,
    sectionText,
  };
})();
