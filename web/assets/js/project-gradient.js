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
 */
function projectThumbnail(project, height) {
  height = height || 200;
  if (project.cover_image) {
    return '<img src="' + project.cover_image + '" class="card-img-top" alt="' + (project.title || '') + '" style="height:' + height + 'px;object-fit:cover">';
  }
  var initials = (project.title || 'P').split(' ').map(function(w) { return w[0]; }).join('').substring(0, 2).toUpperCase();
  return '<div class="card-img-top d-flex align-items-center justify-content-center" style="height:' + height + 'px;background:' + projectGradient(project.title) + '">' +
    '<span style="font-size:3rem;font-weight:700;color:rgba(255,255,255,0.8)">' + initials + '</span></div>';
}
