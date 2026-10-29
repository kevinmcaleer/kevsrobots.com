// Canonical project sections (issue #257).
//
// The one list of section names a project page can have. The editor's `/`
// commands insert these headings, and the page navigation (#259) and the
// view page recognise them, so the spelling is the same everywhere. Add a
// section here, not in the places that use it.
(function () {
  const SECTIONS = [
    {
      id: 'bill-of-materials',
      heading: 'Bill of Materials',
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
    },
    {
      id: '3d-printed-parts',
      heading: '3D printed parts',
      keywords: ['3d', 'printed', 'print', 'stl', 'models', 'cad'],
      icon: 'M8 1.8 13.5 5v6L8 14.2 2.5 11V5zM2.5 5 8 8.2 13.5 5M8 8.2v6',
    },
  ];

  // "Bill of Materials", "bill of materials " and "Bill-of-materials" are
  // the same section.
  function normalise(text) {
    return String(text || '').toLowerCase().replace(/[^a-z0-9]+/g, ' ').trim();
  }

  const byName = new Map(SECTIONS.map(s => [normalise(s.heading), s]));

  window.ProjectSections = {
    list: SECTIONS,
    get: id => SECTIONS.find(s => s.id === id) || null,
    // The canonical section a heading's text names, or null.
    match: text => byName.get(normalise(text)) || null,
  };
})();
