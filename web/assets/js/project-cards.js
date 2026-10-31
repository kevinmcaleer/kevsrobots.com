// Cards in a project's Markdown: a `::: cards` block lays out cards side by
// side, each a picture, a title and some words. A line of `---` starts the
// next card; `:::` ends the block.
//
//   ::: cards
//   ![Open Duck Mini](/odm.jpg)
//   ## Open Duck Mini
//   Created by Antoine Pirrone.
//
//   ---
//
//   ![Microduck](/md.jpg)
//   ## Microduck
//   By Pollen Robotics and Hugging Face.
//   :::
//
// A card's first picture line is its picture and the rest is any Markdown
// (its first heading is its title). `::: cards 3` sets how many across. The
// editor draws and edits these itself (markdown-studio's bundle, cards.ts and
// cardsWidget.ts); the view page renders with marked, so this file adds a
// marked extension that writes the same HTML. The styles are markdown-studio's
// cards.css, vendored beside the editor bundle by
// scripts/update-markdown-editor.sh. Card titles are labels, not page
// sections: view.html leaves headings inside .md-card out of the contents,
// and project-sections.js skips the block's lines. Keep this in step with
// markdown-studio's packages/markdown-editor/src/cards.ts.
(function () {
  const OPEN = /^[ \t]{0,3}:::[ \t]*cards\b[ \t]*(\d*)[ \t]*$/i;
  const CLOSE = /^[ \t]{0,3}:::[ \t]*$/;
  const BREAK = /^[ \t]{0,3}---[ \t]*$/;
  const FENCE = /^[ \t]{0,3}(`{3,}|~{3,})/;
  const PICTURE = /^[ \t]*!\[((?:\\.|[^\]\\])*)\]\(\s*(<[^>\n]*>|[^\s)]+)(?:\s+(?:"([^"]*)"|'([^']*)'))?\s*\)(?:\{([\w-]+)\})?[ \t]*$/;
  const HEADING = /^[ \t]{0,3}(#{1,6})[ \t]+(.*?)(?:[ \t]+#+)?[ \t]*$/;

  function columnsOf(line) {
    const m = OPEN.exec(line);
    if (!m) return null;
    return Math.max(0, Math.min(6, Number(m[1] || 0)));
  }

  // The line of the `:::` closing a block opened at line 0, past any code in it; -1 when it isn't closed.
  function closeOf(lines) {
    let fence = null;
    for (let j = 1; j < lines.length; j++) {
      const marker = FENCE.exec(lines[j]);
      if (fence) {
        if (marker && marker[1][0] === fence[0] && marker[1].length >= fence.length) fence = null;
        continue;
      }
      if (marker) {
        fence = marker[1];
        continue;
      }
      if (CLOSE.test(lines[j])) return j;
      if (OPEN.test(lines[j])) return -1;
    }
    return -1;
  }

  // A block's lines split into its cards' Markdown, at `---` lines outside code.
  function split(body) {
    const cards = [[]];
    let fence = null;
    String(body).split('\n').forEach(line => {
      const marker = FENCE.exec(line);
      if (fence) {
        if (marker && marker[1][0] === fence[0] && marker[1].length >= fence.length) fence = null;
      } else if (marker) fence = marker[1];
      else if (BREAK.test(line)) {
        cards.push([]);
        return;
      }
      cards[cards.length - 1].push(line);
    });
    return cards.map(lines => lines.join('\n').trim());
  }

  function unescapeMarkdown(text) {
    return text.replace(/\\([\\`*_{}[\]()#+\-.!|"'<>~])/g, '$1');
  }

  // { picture: { src, alt, title, size } | null, level, title, text }
  function parse(markdown) {
    let picture = null;
    let level = 2;
    let title = '';
    const rest = [];
    let fence = null;
    String(markdown).split('\n').forEach(line => {
      const marker = FENCE.exec(line);
      if (fence || marker) {
        if (fence && marker && marker[1][0] === fence[0] && marker[1].length >= fence.length) fence = null;
        else if (!fence && marker) fence = marker[1];
        rest.push(line);
        return;
      }
      const pic = picture ? null : PICTURE.exec(line);
      if (pic) {
        picture = {
          src: pic[2].replace(/^<|>$/g, '').trim(),
          alt: unescapeMarkdown(pic[1]).trim(),
          title: (pic[3] || pic[4] || '').trim(),
          size: pic[5] || '',
        };
        return;
      }
      const heading = title ? null : HEADING.exec(line);
      if (heading && heading[2].trim()) {
        level = heading[1].length;
        title = heading[2].trim();
        return;
      }
      rest.push(line);
    });
    return { picture, level, title, text: rest.join('\n').trim() };
  }

  function esc(text) {
    return String(text).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');
  }

  function safeUrl(src) {
    const url = String(src).trim();
    if (/^(javascript|vbscript|file):/i.test(url.replace(/[\s\u0000-\u001f]/g, ''))) return null;
    if (/^data:/i.test(url) && !/^data:image\/(png|gif|jpe?g|webp|avif);/i.test(url)) return null;
    return url;
  }

  // The same markup as cards.ts' cardHtml; `render` turns a card's words into HTML.
  function cardHtml(card, render) {
    const src = card.picture ? safeUrl(card.picture.src) : null;
    const media = src
      ? '<div class="md-card-media' + (card.picture.size === 'cover' ? ' md-card-cover' : '') + '"><img src="' + esc(src) + '" alt="' + esc(card.picture.alt) + '"' +
        (card.picture.title ? ' title="' + esc(card.picture.title) + '"' : '') + ' loading="lazy"></div>'
      : '';
    const words = [card.title ? '#'.repeat(card.level) + ' ' + card.title : '', card.text].filter(Boolean).join('\n\n');
    const body = words ? '<div class="md-card-body">' + render(words) + '</div>' : '';
    return '<div class="md-card' + (!body && !media ? ' md-card-empty' : '') + '">' + media + body + '</div>';
  }

  function html(columns, cards, render) {
    const items = cards.map(markdown => cardHtml(parse(markdown), render)).join('');
    const style = columns ? ' style="--md-cards-columns: ' + columns + '"' : '';
    return '<div class="md-cards' + (columns ? ' md-cards-fixed' : '') + '" data-cards-columns="' + columns + '"' + style + '>' + items + '</div>';
  }

  // marked: a closed `::: cards` block at the start of `src`, or nothing.
  const extension = {
    name: 'cards',
    level: 'block',
    start(src) {
      const m = /^[ \t]{0,3}:::[ \t]*cards\b/im.exec(src);
      return m ? m.index : undefined;
    },
    tokenizer(src) {
      const lines = src.split('\n');
      const columns = columnsOf(lines[0]);
      if (columns === null) return undefined;
      const close = closeOf(lines);
      if (close < 0) return undefined;
      const raw = lines.slice(0, close + 1).join('\n') + (close + 1 < lines.length ? '\n' : '');
      return { type: 'cards', raw, columns, cards: split(lines.slice(1, close).join('\n')) };
    },
    renderer(token) {
      return html(token.columns, token.cards, words => window.marked.parse(words)) + '\n';
    },
  };

  if (window.marked && typeof window.marked.use === 'function') window.marked.use({ extensions: [extension] });

  // The line ranges [open, close] of the cards blocks in some Markdown, for
  // code that reads its headings (project-sections.js) to skip.
  function blocks(markdown) {
    const lines = String(markdown || '').split('\n');
    const out = [];
    let fence = null;
    for (let i = 0; i < lines.length; i++) {
      const marker = FENCE.exec(lines[i]);
      if (fence) {
        if (marker && marker[1][0] === fence[0] && marker[1].length >= fence.length) fence = null;
        continue;
      }
      if (marker) {
        fence = marker[1];
        continue;
      }
      if (columnsOf(lines[i]) === null) continue;
      const close = closeOf(lines.slice(i));
      if (close < 0) continue;
      out.push([i, i + close]);
      i += close;
    }
    return out;
  }

  window.ProjectCards = { parse, split, html, blocks, extension };
})();
