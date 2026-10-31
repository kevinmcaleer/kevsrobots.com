// Embeds in a project's Markdown (issue #257).
//
// A YouTube link on a line of its own plays inline, the way it does in most
// editors. It stays a plain link in the Markdown, so the text still reads
// well anywhere else. `/video` in the editor writes one; the view page and
// the editor's preview both call embedVideos() after rendering.
//
// Videos are a `::: videos` block of YouTube links, one or several, shown
// as a group of thumbnail cards (videoCards), like the video cards across
// the site, on the view page and in the editor's live view:
//
//   ::: videos
//   https://www.youtube.com/watch?v=dQw4w9WgXcQ Part one
//   [Part two](https://youtu.be/abcdefghijk)
//   :::
//
// A line's title is its link text or the words after its address; without
// one, YouTube's title is looked up. The editor draws the block itself
// (markdown-studio's bundle, with videoCards as its videoRenderer); the view
// page renders with marked, so this file adds a marked extension for it.
// Keep the parsing in step with markdown-studio's
// packages/markdown-editor/src/videos.ts.
//
// Older projects' Videos section `| Video | Title |` table shows as the same
// cards, and the table stays the Markdown.
(function () {
  // Same shape as the backend extractor in projects_api/youtube.py —
  // keep the two in sync. Case is preserved (YouTube ids are
  // case-sensitive).
  const YT_ID_RE = /^[A-Za-z0-9_-]{11}$/;

  function youTubeId(s) {
    if (!s) return null;
    const trimmed = String(s).trim();
    if (!trimmed) return null;
    if (YT_ID_RE.test(trimmed)) return trimmed;
    let url;
    try { url = new URL(trimmed.includes('://') ? trimmed : 'https://' + trimmed); }
    catch (_) { return null; }
    let host = url.hostname.toLowerCase();
    if (host.startsWith('www.')) host = host.slice(4);
    let candidate = null;
    if (host === 'youtu.be') {
      candidate = url.pathname.replace(/^\//, '').split('/', 1)[0];
    } else if (host === 'youtube.com' || host === 'm.youtube.com' || host === 'music.youtube.com') {
      const p = url.pathname || '';
      if (p === '/watch' || p.startsWith('/watch/')) {
        candidate = url.searchParams.get('v');
      } else if (p.startsWith('/embed/')) {
        candidate = p.slice('/embed/'.length).split('/', 1)[0];
      } else if (p.startsWith('/shorts/')) {
        candidate = p.slice('/shorts/'.length).split('/', 1)[0];
      } else if (p.startsWith('/v/')) {
        candidate = p.slice('/v/'.length).split('/', 1)[0];
      }
    }
    return (candidate && YT_ID_RE.test(candidate)) ? candidate : null;
  }

  // The Markdown `/video` inserts: the watch URL, on its own line.
  function videoMarkdown(id) {
    return 'https://www.youtube.com/watch?v=' + id;
  }

  // Swaps each paragraph that is nothing but a YouTube URL (a bare one, or
  // a link whose text is its URL) for a 16:9 player.
  function embedVideos(root) {
    root.querySelectorAll('p').forEach(p => {
      const text = p.textContent.trim();
      if (!/^https?:\/\/\S+$/.test(text)) return;
      const kids = p.children;
      if (kids.length > 1 || (kids.length === 1 && kids[0].tagName !== 'A')) return;
      const id = youTubeId(text);
      if (!id || !/^https?:\/\/(www\.|m\.)?(youtube\.com|youtu\.be)\//i.test(text)) return;
      const box = document.createElement('div');
      box.className = 'ratio ratio-16x9 mb-3 project-video-embed';
      const frame = document.createElement('iframe');
      frame.className = 'rounded-1';
      frame.src = 'https://www.youtube.com/embed/' + encodeURIComponent(id);
      frame.title = 'YouTube video';
      frame.setAttribute('frameborder', '0');
      frame.setAttribute('allowfullscreen', '');
      frame.setAttribute('allow', 'accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture');
      frame.loading = 'lazy';
      box.appendChild(frame);
      p.replaceWith(box);
    });
  }

  // The videos in a table's rows (the header row first, each cell's text or
  // Markdown): the YouTube link in the Video column, and the Title column.
  function videoTableItems(rows) {
    const head = (rows[0] || []).map(h => plainText(h).toLowerCase());
    let videoCol = head.findIndex(h => /^(video|videos|url|link|youtube)$/.test(h));
    if (videoCol < 0) videoCol = 0;
    const titleCol = head.findIndex(h => /^(title|name|caption|description)$/.test(h));
    const items = [];
    rows.slice(1).forEach(cells => {
      const text = cells[videoCol] || '';
      // A Markdown link's target, else the cell's first URL, else the cell.
      const link = text.match(/\]\(\s*<?([^)\s>]+)/) || text.match(/https?:\/\/[^\s)|>\]]+/);
      const id = youTubeId(link ? link[1] || link[0] : text);
      if (!id) return;
      items.push({ id, title: titleCol >= 0 ? plainText(cells[titleCol] || '') : '' });
    });
    return items;
  }

  // A cell's Markdown as text: link labels, without emphasis and code marks.
  function plainText(md) {
    return String(md)
      .replace(/!?\[([^\]]*)\]\([^)]*\)/g, '$1')
      .replace(/[*_`~]+/g, '')
      .replace(/\\\|/g, '|')
      .trim();
  }

  // Titles looked up for videos whose Title cell is empty, by id.
  const titles = new Map();

  function lookUpTitle(id) {
    if (!titles.has(id)) {
      const url = 'https://www.youtube.com/oembed?format=json&url=' +
        encodeURIComponent('https://www.youtube.com/watch?v=' + id);
      titles.set(id, fetch(url)
        .then(r => (r.ok ? r.json() : null))
        .then(data => (data && data.title) || '')
        .catch(() => ''));
    }
    return titles.get(id);
  }

  // ------------------------------------------------------- ::: videos

  const VIDEOS_OPEN = /^[ \t]{0,3}:::[ \t]*videos?[ \t]*$/i;
  const VIDEOS_CLOSE = /^[ \t]{0,3}:::[ \t]*$/;
  const LINK = /\[((?:\\.|[^\]\\])*)\]\(\s*<?([^\s)>]+)>?(?:\s+(?:"[^"]*"|'[^']*'))?\s*\)/;
  const URL_IN_TEXT = /<?(https?:\/\/[^\s<>]+|(?:www\.)?(?:youtube\.com|youtu\.be)\/[^\s<>]+)>?/i;

  function cleanTitle(text) {
    return String(text).replace(/[*_`~]+/g, '')
      .replace(/\\([\\`*_{}[\]()#+\-.!|"'<>~])/g, '$1')
      .replace(/^\s*(?:[-*+]|\d+[.)])\s+/, '')
      .replace(/^[\s\-–—:|]+|[\s\-–—:|]+$/g, '')
      .trim();
  }

  // The video on one line of a block, { id, title }, or null.
  function videoFromLine(line) {
    const link = LINK.exec(line);
    if (link) {
      const id = youTubeId(link[2]);
      if (id) {
        const label = cleanTitle(link[1]);
        const rest = cleanTitle(line.replace(link[0], ' '));
        return { id, title: youTubeId(label) ? rest : (label || rest) };
      }
    }
    const bare = URL_IN_TEXT.exec(line);
    if (!bare) return null;
    const id = youTubeId(bare[1]);
    return id ? { id, title: cleanTitle(line.replace(bare[0], ' ')) } : null;
  }

  // The videos in a block's lines, in order.
  function videoBlockItems(body) {
    return String(body).split('\n').map(videoFromLine).filter(Boolean);
  }

  // A block's line for a video: its address, then its title if it has one.
  function videoLine(v) {
    const title = String(v.title || '').replace(/\s+/g, ' ').trim();
    return videoMarkdown(v.id) + (title ? ' ' + title : '');
  }

  // A `::: videos` block of these videos.
  function videoBlock(videos) {
    return '::: videos\n' + videos.map(videoLine).join('\n') + (videos.length ? '\n' : '') + ':::';
  }

  // Every closed `::: videos` block in some Markdown (outside code), as
  // { open, close, videos } with 0-based line numbers.
  function findVideoBlocks(markdown) {
    const lines = String(markdown || '').split('\n');
    const blocks = [];
    let fence = null;
    for (let i = 0; i < lines.length; i++) {
      const line = lines[i];
      const marker = /^[ \t]{0,3}(`{3,}|~{3,})/.exec(line);
      if (fence) {
        if (marker && marker[1][0] === fence[0] && marker[1].length >= fence.length && !line.slice(marker[0].length).trim()) fence = null;
        continue;
      }
      if (marker) { fence = marker[1]; continue; }
      if (!VIDEOS_OPEN.test(line)) continue;
      let close = -1;
      for (let j = i + 1; j < lines.length; j++) {
        if (VIDEOS_CLOSE.test(lines[j])) { close = j; break; }
        if (/^[ \t]{0,3}:::/.test(lines[j])) break;
      }
      if (close < 0) continue;
      blocks.push({ open: i, close, videos: videoBlockItems(lines.slice(i + 1, close).join('\n')) });
      i = close;
    }
    return blocks;
  }

  function escAttr(text) {
    return String(text).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');
  }

  // marked: a closed `::: videos` block at the start of `src` becomes a
  // placeholder that renderVideoBlocks() fills with the cards.
  const markedExtension = {
    name: 'videos',
    level: 'block',
    start(src) {
      const m = /^[ \t]{0,3}:::[ \t]*videos?[ \t]*$/im.exec(src);
      return m ? m.index : undefined;
    },
    tokenizer(src) {
      const lines = src.split('\n');
      if (!VIDEOS_OPEN.test(lines[0])) return undefined;
      for (let i = 1; i < lines.length; i++) {
        if (VIDEOS_CLOSE.test(lines[i])) {
          const raw = lines.slice(0, i + 1).join('\n') + (i + 1 < lines.length ? '\n' : '');
          return { type: 'videos', raw, videos: videoBlockItems(lines.slice(1, i).join('\n')) };
        }
        if (/^[ \t]{0,3}:::/.test(lines[i])) return undefined;
      }
      return undefined;
    },
    renderer(token) {
      return '<div class="project-videos-block" data-videos="' + escAttr(JSON.stringify(token.videos)) + '"></div>\n';
    },
  };
  if (window.marked && typeof window.marked.use === 'function') window.marked.use({ extensions: [markedExtension] });

  // Swaps each `::: videos` placeholder under `root` for its cards, which
  // play in place. Returns how many blocks there were.
  function renderVideoBlocks(root) {
    const blocks = root.querySelectorAll('.project-videos-block');
    blocks.forEach(el => {
      let videos = [];
      try { videos = JSON.parse(el.dataset.videos || '[]'); } catch (_) { /* none */ }
      if (!videos.length) { el.remove(); return; }
      el.replaceWith(videoCards(videos, { play: true }));
    });
    return blocks.length;
  }

  // A video's card swaps its thumbnail for the playing video.
  function playInCard(col, id, title) {
    const thumb = col.querySelector('.project-video-thumb');
    if (!thumb || thumb.querySelector('iframe')) return;
    const frame = document.createElement('iframe');
    frame.src = 'https://www.youtube.com/embed/' + encodeURIComponent(id) + '?autoplay=1';
    frame.title = title || 'YouTube video';
    frame.setAttribute('frameborder', '0');
    frame.setAttribute('allowfullscreen', '');
    frame.setAttribute('allow', 'accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture');
    thumb.replaceChildren(frame);
  }

  // A gallery of cards, one per video: the YouTube thumbnail with the title
  // underneath, linking to the video. A video without a title shows
  // YouTube's title once it's looked up ("YouTube video" until then).
  //
  // With { play: true } (the view page) a click plays the video in its card;
  // otherwise the card links to YouTube.
  function videoCards(items, opts) {
    const play = !!(opts && opts.play);
    const grid = document.createElement('div');
    grid.className = 'row row-cols-1 row-cols-sm-2 row-cols-lg-3 g-3 mb-3 project-video-grid';
    items.forEach(v => {
      const col = document.createElement('div');
      col.className = 'col';
      const link = document.createElement('a');
      link.className = 'text-decoration-none';
      link.href = videoMarkdown(v.id);
      link.target = '_blank';
      link.rel = 'noopener';
      const card = document.createElement('div');
      card.className = 'card h-100 card-hover shadow-hover bg-light rounded-3 border-0 project-video-card';
      const thumb = document.createElement('div');
      thumb.className = 'project-video-thumb';
      const img = document.createElement('img');
      img.className = 'card-img-top';
      img.src = 'https://img.youtube.com/vi/' + encodeURIComponent(v.id) + '/mqdefault.jpg';
      img.loading = 'lazy';
      img.alt = '';
      const play = document.createElement('i');
      play.className = 'fa-brands fa-youtube project-video-play';
      play.setAttribute('aria-hidden', 'true');
      thumb.append(img, play);
      const body = document.createElement('div');
      body.className = 'card-body';
      const title = document.createElement('p');
      title.className = 'black text-truncate-3-lines mb-0';
      title.textContent = v.title || 'YouTube video';
      if (!v.title) {
        lookUpTitle(v.id).then(t => {
          if (t) { title.textContent = t; img.alt = t; }
        });
      } else img.alt = v.title;
      body.appendChild(title);
      card.append(thumb, body);
      link.appendChild(card);
      col.appendChild(link);
      if (play) {
        link.addEventListener('click', e => {
          if (e.button !== 0 || e.metaKey || e.ctrlKey || e.shiftKey) return;
          e.preventDefault();
          playInCard(col, v.id, title.textContent);
        });
      }
      grid.appendChild(col);
    });
    return grid;
  }

  window.ProjectEmbeds = {
    youTubeId, videoMarkdown, embedVideos, videoTableItems, videoCards,
    videoFromLine, videoBlockItems, videoLine, videoBlock, findVideoBlocks, renderVideoBlocks, markedExtension,
  };
})();
