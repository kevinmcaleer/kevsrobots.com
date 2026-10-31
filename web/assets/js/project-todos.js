// `#todo` notes to self, kept off the published page.
//
// In the project editor a writer can leave `#todo add the wiring photo`
// anywhere; the editor (markdown-studio's bundle, todos.ts) lists them in its
// "N to do" box, and one click there takes the note out. Any still in the
// text when the project is viewed are the writer's own reminders, so
// view.html calls ProjectTodos.strip() on the Markdown before rendering it.
// Keep the rules in step with findTodos()/completeTodo() in todos.ts:
//
// - the tag is `#todo` (any case), not part of a longer word or a URL;
// - its note runs to the end of the line, the next `#todo`, or (in a table
//   row) the end of the cell;
// - `#todo` in a fenced code block or an inline code span is left alone;
// - a line left with nothing on it (or only a list marker) goes too.
(function () {
  const TAG = /(?<![\w#/&@\\])#todo(?![\w-])/gi;
  const FENCE = /^ {0,3}(`{3,}|~{3,})/;
  const MARKER = /^\s*(?:[-*+] \[[ xX]\]|[-*+]|\d+[.)]|>)?\s*$/;

  // Whether `pos` on `text` is inside an inline code span.
  function inCode(text, pos) {
    let open = null;
    for (let i = 0; i < pos; i++) {
      if (text[i] === '\\' && !open) {
        i++;
        continue;
      }
      if (text[i] !== '`') continue;
      let run = 1;
      while (text[i + run] === '`') run++;
      const ticks = text.slice(i, i + run);
      if (open === null) open = ticks;
      else if (open === ticks) open = null;
      i += run - 1;
    }
    return open !== null && text.indexOf(open, pos) >= 0;
  }

  // The [from, to) of each todo (tag and note) on a line.
  function todosOn(text) {
    const out = [];
    const tableRow = /^\s*\|/.test(text);
    TAG.lastIndex = 0;
    for (let m = TAG.exec(text); m; m = TAG.exec(text)) {
      const at = m.index;
      const afterTag = at + m[0].length;
      let end = text.length;
      TAG.lastIndex = afterTag;
      const next = TAG.exec(text);
      if (next) end = next.index;
      if (tableRow) {
        let pipe = afterTag;
        while ((pipe = text.indexOf('|', pipe)) >= 0 && text[pipe - 1] === '\\') pipe++;
        if (pipe >= 0 && pipe < end) end = pipe;
      }
      TAG.lastIndex = next ? next.index : text.length;
      if (inCode(text, at)) continue;
      out.push([at, afterTag + text.slice(afterTag, end).trimEnd().length]);
    }
    return out;
  }

  // The line without its todos, or null when nothing worth keeping is left.
  function stripLine(text) {
    const todos = todosOn(text);
    if (!todos.length) return text;
    let line = text;
    for (let i = todos.length - 1; i >= 0; i--) {
      let [from, to] = todos[i];
      const before = line.slice(0, from);
      const after = line.slice(to);
      if (MARKER.test(before) && !after.trim()) return null;
      const space = /[ \t]+$/.exec(before);
      if (space) from -= space[0].length;
      else if (/^[ \t]+/.test(after)) to += /^[ \t]+/.exec(after)[0].length;
      if (MARKER.test(before) && before.trim() && space) from += 1;
      line = line.slice(0, from) + line.slice(to);
    }
    return line;
  }

  // The Markdown with every `#todo` note taken out.
  function strip(markdown) {
    if (!markdown || !/#todo/i.test(markdown)) return markdown;
    const out = [];
    let fence = null;
    for (const text of markdown.split('\n')) {
      const f = FENCE.exec(text);
      if (fence) {
        if (f && f[1][0] === fence[0] && f[1].length >= fence.length && !text.slice(f[0].length).trim()) fence = null;
        out.push(text);
        continue;
      }
      if (f) {
        fence = f[1];
        out.push(text);
        continue;
      }
      const line = stripLine(text);
      if (line !== null) out.push(line);
    }
    return out.join('\n');
  }

  window.ProjectTodos = { strip };
})();
