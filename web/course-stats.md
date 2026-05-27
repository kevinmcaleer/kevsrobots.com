---
title: Course Analytics
description: Per-course funnel and completion analytics for KevsRobots.com courses
layout: content
sitemap: false
---

<style>
  .course-stats-summary {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
    gap: 1rem;
    margin: 1.5rem 0;
  }
  .course-stats-summary .stat {
    background: var(--card-bg, #1e1e1e);
    border: 1px solid var(--border-color, #333);
    border-radius: 8px;
    padding: 1rem;
    text-align: center;
  }
  .course-stats-summary .stat .num {
    font-size: 1.6rem;
    font-weight: 700;
    color: var(--accent-color, #f39c12);
  }
  .course-stats-summary .stat .lbl {
    font-size: 0.85rem;
    opacity: 0.75;
  }
  table.course-funnel {
    width: 100%;
    border-collapse: collapse;
    margin-top: 1rem;
    font-size: 0.92rem;
  }
  table.course-funnel th, table.course-funnel td {
    padding: 0.5rem 0.6rem;
    border-bottom: 1px solid var(--border-color, #333);
    text-align: right;
  }
  table.course-funnel th:first-child,
  table.course-funnel td:first-child { text-align: left; }
  table.course-funnel tbody tr:hover { background: rgba(255,255,255,0.04); cursor: pointer; }
  table.course-funnel th { cursor: pointer; user-select: none; }
  table.course-funnel th .sort-ind { opacity: 0.4; font-size: 0.7em; }
  .completion-bar {
    display: inline-block;
    width: 80px;
    height: 8px;
    background: rgba(255,255,255,0.08);
    border-radius: 4px;
    overflow: hidden;
    vertical-align: middle;
    margin-right: 6px;
  }
  .completion-bar > span {
    display: block;
    height: 100%;
    background: linear-gradient(90deg, #e74c3c 0%, #f39c12 50%, #2ecc71 100%);
  }
  .lesson-detail {
    background: var(--card-bg, #1e1e1e);
    border: 1px solid var(--border-color, #333);
    border-radius: 8px;
    padding: 1rem 1.2rem;
    margin: 1rem 0 2rem;
  }
  .lesson-detail h3 { margin-top: 0; }
  .lesson-bar {
    display: grid;
    grid-template-columns: 2.5rem 1fr 4rem;
    gap: 0.5rem;
    align-items: center;
    padding: 2px 0;
    font-size: 0.85rem;
  }
  .lesson-bar .idx { opacity: 0.5; text-align: right; }
  .lesson-bar .bar {
    height: 16px;
    background: rgba(255,255,255,0.04);
    border-radius: 3px;
    position: relative;
    overflow: hidden;
  }
  .lesson-bar .bar > span {
    display: block;
    height: 100%;
    background: var(--accent-color, #f39c12);
    opacity: 0.85;
  }
  .lesson-bar .bar > .label {
    position: absolute;
    left: 6px;
    top: 0;
    line-height: 16px;
    font-size: 0.75rem;
    color: #fff;
    text-shadow: 0 0 3px rgba(0,0,0,0.7);
    background: none;
    height: auto;
  }
  .lesson-bar .hits { text-align: right; font-variant-numeric: tabular-nums; }
  .lesson-bar.peak .bar > span { background: #2ecc71; }
  .lesson-bar.seo-jackpot { background: rgba(46,204,113,0.08); border-left: 2px solid #2ecc71; padding-left: 4px; }
  .caveat {
    background: rgba(243, 156, 18, 0.08);
    border-left: 3px solid #f39c12;
    padding: 0.75rem 1rem;
    margin: 1.5rem 0;
    font-size: 0.9rem;
  }
  #course-stats-loading { opacity: 0.6; font-style: italic; }
</style>

<div class="content">
  <h1>Course Analytics</h1>
  <p class="lead">Per-course funnel: how many people enter each course and where they drop off.</p>

  <div class="caveat">
    <strong>Methodology note:</strong> Numbers are <em>cumulative anonymous page hits</em> from
    <code>page_count.kevsrobots.com</code>, not unique-user journeys. &ldquo;Completion %&rdquo; is a
    <em>proxy</em> &mdash; final lesson hits divided by intro hits &mdash; not a true session-tracked
    completion rate. Lessons that out-perform the intro are usually SEO landing pages (people arriving
    direct from Google mid-course); they're flagged in green.
  </div>

  <div id="course-stats-summary" class="course-stats-summary">
    <div id="course-stats-loading">Loading live page counts&hellip;</div>
  </div>

  <h2>Courses</h2>
  <p style="font-size:0.85em;opacity:0.7;">Click any row to expand the per-lesson drop-off curve. Click column headers to sort.</p>
  <table class="course-funnel" id="course-funnel-table">
    <thead>
      <tr>
        <th data-sort="name">Course <span class="sort-ind">⇅</span></th>
        <th data-sort="lessons">Lessons <span class="sort-ind">⇅</span></th>
        <th data-sort="intro">Intro <span class="sort-ind">⇅</span></th>
        <th data-sort="total">Total Hits <span class="sort-ind">▼</span></th>
        <th data-sort="final">Final <span class="sort-ind">⇅</span></th>
        <th data-sort="completion">Completion <span class="sort-ind">⇅</span></th>
      </tr>
    </thead>
    <tbody><tr><td colspan="6" style="opacity:0.6;">Loading&hellip;</td></tr></tbody>
  </table>
</div>

<script id="course-lessons-data" type="application/json">
{{ site.data.course_lessons | jsonify }}
</script>

<script>
(function() {
  const PAGE_COUNT_URL = 'https://page_count.kevsrobots.com/stats';
  const courses = JSON.parse(document.getElementById('course-lessons-data').textContent);

  const toInt = (v) => parseInt(String(v).replace(/,/g, ''), 10) || 0;

  fetch(PAGE_COUNT_URL)
    .then(r => r.json())
    .then(data => render(data))
    .catch(err => {
      document.getElementById('course-stats-loading').textContent =
        'Failed to load page-count data: ' + err.message;
    });

  function render(pc) {
    // Build hits lookup: path -> count
    const hits = {};
    for (const [path, count] of Object.entries(pc.popular_pages || {})) {
      hits['/' + path.replace(/^\/+/, '')] = toInt(count);
    }
    const lookupHits = (url) => hits[url] || hits[url.replace(/\.html$/, '')] || 0;

    // Augment courses
    const rows = courses.map(c => {
      const perLesson = c.lessons.map(l => ({ ...l, hits: lookupHits(l.url) }));
      const intro = perLesson[0]?.hits || 0;
      const final = perLesson[perLesson.length - 1]?.hits || 0;
      const total = perLesson.reduce((s, l) => s + l.hits, 0);
      const completion = intro > 0 ? (final / intro * 100) : 0;
      return { ...c, perLesson, intro, final, total, completion };
    }).filter(r => r.total > 0);

    // Summary tiles
    const sumEl = document.getElementById('course-stats-summary');
    const totalHits = rows.reduce((s, r) => s + r.total, 0);
    const avgCompletion = rows.filter(r => r.intro > 0)
      .reduce((s, r, _, a) => s + r.completion / a.length, 0);
    const totalCourses = rows.length;
    const totalLessons = rows.reduce((s, r) => s + r.lessons.length, 0);
    sumEl.innerHTML = [
      tile(totalCourses, 'Active Courses'),
      tile(totalLessons.toLocaleString(), 'Lessons Tracked'),
      tile(totalHits.toLocaleString(), 'Total Lesson Hits'),
      tile(avgCompletion.toFixed(1) + '%', 'Avg Completion Proxy'),
      tile(toInt(pc.total_visits).toLocaleString(), 'Site Total Visits'),
      tile(toInt(pc.unique_visitors).toLocaleString(), 'Unique Visitors'),
    ].join('');

    // Render table
    let sortKey = 'total';
    let sortDir = -1;
    const tbody = document.querySelector('#course-funnel-table tbody');

    function renderTable() {
      const sorted = rows.slice().sort((a, b) => {
        const va = a[sortKey], vb = b[sortKey];
        if (typeof va === 'string') return sortDir * va.localeCompare(vb);
        return sortDir * (va - vb);
      });
      tbody.innerHTML = sorted.map(r => `
        <tr data-slug="${r.slug}">
          <td><a href="${r.lessons[0].url}">${escapeHtml(r.name)}</a></td>
          <td>${r.lessons.length}</td>
          <td>${r.intro.toLocaleString()}</td>
          <td>${r.total.toLocaleString()}</td>
          <td>${r.final.toLocaleString()}</td>
          <td>
            <span class="completion-bar"><span style="width:${Math.min(100, r.completion).toFixed(0)}%"></span></span>
            ${r.completion.toFixed(1)}%
          </td>
        </tr>
        <tr class="lesson-detail-row" id="detail-${r.slug}" style="display:none;">
          <td colspan="6">${renderLessonDetail(r)}</td>
        </tr>
      `).join('');

      // Wire row clicks
      tbody.querySelectorAll('tr[data-slug]').forEach(tr => {
        tr.addEventListener('click', () => {
          const slug = tr.getAttribute('data-slug');
          const row = document.getElementById('detail-' + slug);
          row.style.display = row.style.display === 'none' ? '' : 'none';
        });
      });
    }

    function renderLessonDetail(r) {
      const peak = Math.max(...r.perLesson.map(l => l.hits), 1);
      const introHits = r.intro || 1;
      const bars = r.perLesson.map((l, i) => {
        const pct = (l.hits / peak * 100).toFixed(1);
        const isPeak = l.hits === peak;
        const isJackpot = l.hits > introHits * 1.5 && i > 0;
        const cls = ['lesson-bar', isPeak ? 'peak' : '', isJackpot ? 'seo-jackpot' : ''].join(' ');
        const note = isJackpot ? ' &nbsp; <span style="color:#2ecc71;font-size:0.75em;">⭐ SEO entry</span>' : '';
        return `
          <div class="${cls}">
            <div class="idx">${String(i).padStart(2, '0')}</div>
            <div class="bar">
              <span style="width:${pct}%"></span>
              <span class="label"><a href="${l.url}" style="color:inherit;text-decoration:none;">${escapeHtml(l.file)}</a>${note}</span>
            </div>
            <div class="hits">${l.hits.toLocaleString()}</div>
          </div>`;
      }).join('');
      return `<div class="lesson-detail">
        <h3>${escapeHtml(r.name)} <small style="opacity:0.6;font-weight:normal;">(${r.lessons.length} lessons, ${r.completion.toFixed(1)}% completion proxy)</small></h3>
        ${bars}
      </div>`;
    }

    // Column sort
    document.querySelectorAll('#course-funnel-table th[data-sort]').forEach(th => {
      th.addEventListener('click', () => {
        const k = th.getAttribute('data-sort');
        if (sortKey === k) sortDir *= -1;
        else { sortKey = k; sortDir = (k === 'name') ? 1 : -1; }
        document.querySelectorAll('#course-funnel-table th .sort-ind').forEach(s => s.textContent = '⇅');
        th.querySelector('.sort-ind').textContent = sortDir === 1 ? '▲' : '▼';
        renderTable();
      });
    });

    renderTable();
  }

  function tile(num, lbl) {
    return `<div class="stat"><div class="num">${num}</div><div class="lbl">${lbl}</div></div>`;
  }
  function escapeHtml(s) {
    return String(s).replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  }
})();
</script>
