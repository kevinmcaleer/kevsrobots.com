/**
 * Generate a unique gradient background from a project title.
 * Returns a CSS linear-gradient string.
 */
function projectGradient(title) {
  var hash = 0;
  for (var i = 0; i < (title || '').length; i++) {
    hash = ((hash << 5) - hash) + title.charCodeAt(i);
    hash = hash & hash;
  }
  var h1 = Math.abs(hash % 360);
  var h2 = (h1 + 40 + Math.abs((hash >> 8) % 60)) % 360;
  var angle = 120 + Math.abs((hash >> 16) % 60);
  return 'linear-gradient(' + angle + 'deg, hsl(' + h1 + ',70%,65%), hsl(' + h2 + ',70%,55%))';
}

/**
 * Returns HTML for a project card thumbnail.
 * Uses cover_image if available, otherwise a generated gradient with initials.
 *
 * The thumbnail is always rendered at a 4:3 aspect ratio via the
 * `.kr-project-card .card-img-top { aspect-ratio: 4/3 }` rule in
 * main.scss. The optional `height` param is kept for back-compat with
 * callers that haven't migrated to the .kr-project-card wrapper yet —
 * it falls back to a fixed pixel height for those.
 */
function projectThumbnail(project, height) {
  // When the card uses .kr-project-card (recommended), the height is
  // controlled by CSS aspect-ratio. We still emit a fallback inline
  // style for legacy callers that don't wrap the card in
  // .kr-project-card — they get the previous fixed-height behaviour.
  var style = (height ? 'height:' + height + 'px;' : '') + 'object-fit:cover;width:100%;background:#f1f3f5;';
  if (project.cover_image) {
    return '<img src="' + project.cover_image + '" class="card-img-top" loading="lazy" alt="' + (project.title || '') + '" style="' + style + '">';
  }
  var initials = (project.title || 'P').split(' ').map(function(w) { return w[0]; }).join('').substring(0, 2).toUpperCase();
  var fallbackStyle = (height ? 'height:' + height + 'px;' : '') + 'width:100%;background:' + projectGradient(project.title) + ';';
  return '<div class="card-img-top d-flex align-items-center justify-content-center" style="' + fallbackStyle + '">' +
    '<span style="font-size:3rem;font-weight:700;color:rgba(255,255,255,0.8)">' + initials + '</span></div>';
}
