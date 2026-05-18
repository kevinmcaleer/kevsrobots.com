/**
 * Shared helpers for the Projects Hub and Search pages.
 * Depends on project-gradient.js for projectThumbnail().
 */
var ProjectSearch = (function () {
  var DIFFICULTY_COLORS = {
    beginner: 'success',
    intermediate: 'warning',
    advanced: 'danger'
  };

  var STATUS_BADGES = {
    wip: '<span class="badge bg-info">🔨 WIP</span>',
    completed: '<span class="badge bg-success">✅ Completed</span>'
  };

  function esc(text) {
    var d = document.createElement('div');
    d.textContent = text == null ? '' : String(text);
    return d.innerHTML;
  }

  /**
   * Rank projects for the hub: completed projects come first (by created_at desc),
   * then everything else by created_at desc, with a secondary nudge for projects
   * updated recently relative to their creation.
   */
  function rankHubProjects(projects) {
    var list = (projects || []).slice();
    list.sort(function (a, b) {
      var aDone = a.status === 'completed' ? 1 : 0;
      var bDone = b.status === 'completed' ? 1 : 0;
      if (aDone !== bDone) return bDone - aDone;
      var aCreated = new Date(a.created_at || 0).getTime();
      var bCreated = new Date(b.created_at || 0).getTime();
      if (bCreated !== aCreated) return bCreated - aCreated;
      var aUpdated = new Date(a.updated_at || a.created_at || 0).getTime();
      var bUpdated = new Date(b.updated_at || b.created_at || 0).getTime();
      return bUpdated - aUpdated;
    });
    return list;
  }

  /**
   * Render a single project card matching the hub layout.
   * `myProjectIds` is an optional Set of project ids owned by the viewer.
   */
  function cardHtml(p, myProjectIds) {
    var owned = myProjectIds && myProjectIds.has(p.id);
    var href = owned ? '/projects/editor.html?id=' + p.id : '/projects/view.html?id=' + p.id;
    var diff = p.difficulty
      ? '<span class="badge bg-' + (DIFFICULTY_COLORS[p.difficulty] || 'secondary') + '">' + esc(p.difficulty) + '</span>'
      : '';
    var time = p.estimated_minutes
      ? '<span class="badge bg-light text-dark"><i class="fas fa-clock"></i> ' + p.estimated_minutes + 'min</span>'
      : '';
    var remixBadge = p.is_remix
      ? '<span class="badge bg-light text-muted"><i class="fas fa-code-branch"></i> Remix</span>'
      : '';
    var remixIcon = p.is_remix
      ? '<i class="fas fa-code-branch text-muted me-1" title="Remix of another project" data-bs-toggle="tooltip"></i>'
      : '';
    var tagsHtml = (p.tags || []).slice(0, 4)
      .map(function (t) { return '<span class="badge bg-light text-primary">' + esc(t) + '</span>'; })
      .join('');
    return (
      '<div class="col">' +
        '<a href="' + href + '" class="text-decoration-none">' +
          '<div class="card h-100 border-0 shadow-sm card-hover">' +
            projectThumbnail(p, 200) +
            '<div class="card-body">' +
              '<h5 class="card-title text-dark">' + remixIcon + esc(p.title) + '</h5>' +
              '<p class="card-text text-muted small">' + esc(p.short_description || '') + '</p>' +
              '<div class="d-flex flex-wrap gap-1 mb-2">' +
                (STATUS_BADGES[p.status] || '') +
                diff + time + remixBadge +
              '</div>' +
              '<div class="d-flex flex-wrap gap-1">' + tagsHtml + '</div>' +
            '</div>' +
            '<div class="card-footer bg-transparent border-0 d-flex justify-content-between align-items-center">' +
              '<small class="text-muted">by ' + esc(p.author_username) + ' &middot; ' +
                new Date(p.created_at).toLocaleDateString() + '</small>' +
              '<small class="text-muted d-flex gap-2 align-items-center">' +
                '<span id="card-makes-' + p.id + '" class="d-none"><i class="fas fa-hammer"></i> <span data-count></span></span>' +
                '<span id="card-likes-' + p.id + '"><i class="far fa-heart"></i> </span>' +
                '<span title="Total downloads"><i class="fas fa-download"></i> ' + (p.download_count || 0) + '</span>' +
              '</small>' +
            '</div>' +
          '</div>' +
        '</a>' +
      '</div>'
    );
  }

  /**
   * Attach like + makes counts to a freshly rendered list of cards.
   * Best-effort: failures leave counts hidden / at 0.
   */
  function attachCounts(api, projects) {
    if (typeof ProjectInteractions !== 'undefined') {
      projects.forEach(function (p) {
        var url = 'projects/view.html?id=' + p.id;
        ProjectInteractions.getLikeCount(url).then(function (data) {
          var el = document.getElementById('card-likes-' + p.id);
          if (el && data.count > 0) {
            el.innerHTML = '<i class="far fa-heart"></i> ' + data.count;
          }
        });
      });
    }
    projects.forEach(function (p) {
      fetch(api + '/api/projects/' + p.id + '/makes')
        .then(function (r) { return r.ok ? r.json() : []; })
        .then(function (makes) {
          if (!makes || makes.length === 0) return;
          var wrap = document.getElementById('card-makes-' + p.id);
          if (!wrap) return;
          wrap.querySelector('[data-count]').textContent = makes.length;
          wrap.classList.remove('d-none');
        })
        .catch(function () {});
    });
  }

  return {
    esc: esc,
    cardHtml: cardHtml,
    rankHubProjects: rankHubProjects,
    attachCounts: attachCounts,
    DIFFICULTY_COLORS: DIFFICULTY_COLORS,
    STATUS_BADGES: STATUS_BADGES
  };
})();
