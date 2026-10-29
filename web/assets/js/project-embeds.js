// Embeds in a project's Markdown (issue #257).
//
// A YouTube link on a line of its own plays inline, the way it does in most
// editors. It stays a plain link in the Markdown, so the text still reads
// well anywhere else. `/video` in the editor writes one; the view page and
// the editor's preview both call embedVideos() after rendering.
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

  window.ProjectEmbeds = { youTubeId, videoMarkdown, embedVideos };
})();
