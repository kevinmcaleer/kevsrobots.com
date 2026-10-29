// Callouts in a project's Markdown: Obsidian's `> [!type] Title` boxes.
//
//   > [!warning] Mind the mains
//   > Unplug it before you open the case.
//
// A `+` or `-` after the type makes one foldable (open or closed to start
// with). The editor's preview renders them itself (markdown-studio's bundle);
// the view page renders with marked, which leaves them as quotes, so
// render() turns those quotes into the same markup afterwards. The styles are
// markdown-studio's callouts.css, vendored beside the editor bundle by
// scripts/update-markdown-editor.sh. Keep TYPES in step with calloutKinds in
// markdown-studio's packages/markdown-editor/src/callouts.ts.
(function () {
  const TYPES = {
    note: ['Note'],
    abstract: ['Abstract', 'summary', 'tldr'],
    info: ['Info', 'information'],
    todo: ['Todo'],
    tip: ['Tip', 'hint', 'important'],
    success: ['Success', 'check', 'done'],
    question: ['Question', 'help', 'faq'],
    warning: ['Warning', 'caution', 'attention', 'alert'],
    failure: ['Failure', 'fail', 'missing'],
    danger: ['Danger', 'error'],
    bug: ['Bug'],
    example: ['Example'],
    quote: ['Quote', 'cite'],
  };
  const BY_NAME = {};
  Object.keys(TYPES).forEach(type => {
    BY_NAME[type] = type;
    TYPES[type].slice(1).forEach(alias => { BY_NAME[alias] = type; });
  });
  const HEADER_RE = /^\s*\[!([\w-]+)\]([+-]?)[ \t]*/;

  // The callout a header names: its type (a note for one we don't know),
  // what was written, and the title to show when it has none of its own.
  function kind(name) {
    const lower = name.toLowerCase();
    const type = BY_NAME[lower] || 'note';
    const label = BY_NAME[lower] ? TYPES[type][0] : lower.charAt(0).toUpperCase() + lower.slice(1).replace(/-/g, ' ');
    return { type: type, name: lower, label: label };
  }

  // Moves the first line of `p` (up to a newline or <br>) into `into`.
  function takeFirstLine(p, into) {
    while (p.firstChild) {
      const node = p.firstChild;
      if (node.nodeName === 'BR') { node.remove(); return; }
      if (node.nodeType === Node.TEXT_NODE && node.data.includes('\n')) {
        const at = node.data.indexOf('\n');
        into.append(node.data.slice(0, at));
        node.data = node.data.slice(at + 1);
        return;
      }
      into.append(node);
    }
  }

  function convert(quote) {
    const p = quote.firstElementChild;
    if (!p || p.tagName !== 'P') return;
    const first = p.firstChild;
    if (!first || first.nodeType !== Node.TEXT_NODE) return;
    const match = HEADER_RE.exec(first.data);
    if (!match) return;
    first.data = first.data.slice(match[0].length);

    const info = kind(match[1]);
    const fold = match[2];
    const box = document.createElement(fold ? 'details' : 'div');
    box.className = 'callout callout-' + info.type;
    box.dataset.callout = info.name;
    if (fold) {
      box.dataset.calloutFold = fold;
      box.open = fold === '+';
    }

    const title = document.createElement(fold ? 'summary' : 'div');
    title.className = 'callout-title';
    const icon = document.createElement('span');
    icon.className = 'callout-icon';
    icon.setAttribute('aria-hidden', 'true');
    const text = document.createElement('span');
    text.className = 'callout-title-text';
    takeFirstLine(p, text);
    if (!text.textContent.trim()) text.textContent = info.label;
    title.append(icon, text);

    const content = document.createElement('div');
    content.className = 'callout-content';
    if (!p.textContent.trim() && !p.querySelector('img')) p.remove();
    while (quote.firstChild) content.append(quote.firstChild);
    // A title-only callout: emptied, so the stylesheet's :empty hides the body.
    if (!content.textContent.trim() && !content.querySelector('*')) content.textContent = '';

    box.append(title, content);
    quote.replaceWith(box);
  }

  // Turns each `> [!type]` quote under `root` into a callout. Nested ones
  // too; quotes that aren't callouts are left as they are.
  function render(root) {
    root.querySelectorAll('blockquote').forEach(convert);
  }

  window.ProjectCallouts = { render: render };
})();
