# Claude Rules for KevsRobots.com

This file contains guidelines for working with the KevsRobots.com repository.

---

## Repository Structure

- **`/web`** - Jekyll site source files
  - `_posts/` - Blog posts (format: `YYYY-MM-DD-title.md`)
  - `_layouts/` - Jekyll layout templates
  - `_includes/` - Reusable components (header, footer, etc.)
  - `_site/` - Generated static site (DO NOT edit directly)
  - `_config.yml` - Jekyll configuration
- **`/source`** - Course content (Markdown lessons)
- **`/stacks`** - Docker configurations for deployment
- **`build.py`** - Builds courses from `/source` to `/web/learn`

---

## Deployment Architecture

### Local Development
- Jekyll runs in Docker container on port 4000
- Uses `docker-compose` from `/stacks/docker-compose.yml`
- Auto-rebuilds on file changes

### Production
- 4x Raspberry Pi 5 nodes running Docker containers
- Nginx load balancer distributes traffic
- Images pushed to Pi-hosted registry at `192.168.2.1:5000`
- Cloudflare CDN in front (handles SSL, caching, health checks)

### Build & Deploy Workflow
1. **Develop locally** - Edit files, test at `localhost:4000`
2. **Commit to GitHub** - Push changes to `kevinmcaleer/kevsrobots.com`
3. **Build on Pi** - `docker build` pulls latest from GitHub
4. **Push to registry** - `docker push 192.168.2.1:5000/kevsrobots:latest`
5. **Deploy to nodes** - Each Pi pulls and restarts containers

---

## Important Rules

### Docker Builds
- **NEVER use `--no-cache`** - The Dockerfile has automatic cache busting via GitHub API
- Build command: `docker build -t 192.168.2.1:5000/kevsrobots:latest .`
- The Dockerfile fetches latest commit hash to invalidate cache only when needed
- This keeps gem installation and other layers cached for speed

### Jekyll/HTML
- **Footer must be inside `<body>` tag** - Check `_layouts/default.html`
- **All `<script>` tags must have closing `</script>`** - No self-closing
- **HTML compression** - Currently disabled in `_config.yml`, can re-enable with conservative settings
- **Images** - Use `loading="lazy"` attribute for performance
- **Build time** - ~60 seconds for full site rebuild

### Blog Posts
- **File naming** - `YYYY-MM-DD-title.md` in `/web/_posts/`
- **Frontmatter required** - title, description, layout, date, author, cover, tags
- **Tone** - Friendly, maker-focused, conversational (start with "Ahoy there makers!")
- **Images** - Place in `/web/assets/img/blog/post-name/`
- **Video embeds** - Use `videos:` frontmatter with YouTube IDs

### Courses
- **Source** - Edit in `/source/course-name/` as Markdown
- **Build** - Run `python build.py` to generate `/web/learn/` content
- **Naming** - Use `00_intro.md`, `01_lesson.md`, etc.
- **Structure** - Each course needs `course.yml` metadata file

### Nginx Configuration
- **Backend** - `/stacks/kevsrobots/nginx-optimized.conf`
  - Timeouts: 60s (for large images)
  - Gzip: OFF (Cloudflare handles compression)
  - Sendfile: ON with 512KB chunks
- **Load Balancer** - `/stacks/nginx/nginx.conf`
  - Proxy timeouts: 60s
  - Buffers: 16x8k
  - Has `/health` endpoint for Cloudflare

### Performance
- **Image optimization** - Run `/optimize_images.py` manually when adding new images
- **Lazy loading** - Enabled via `_includes/lazy_load.html`
- **Caching** - Cloudflare handles at edge, not nginx
- **Health checks** - Container healthcheck uses `127.0.0.1` not `localhost` (IPv4 vs IPv6 issue)

---

## Common Tasks

### Add a New Blog Post
```bash
# 1. Create file
cd web/_posts
touch YYYY-MM-DD-post-title.md

# 2. Add frontmatter (see existing posts for template)
# 3. Write content in Markdown
# 4. Add images to /web/assets/img/blog/post-title/
# 5. Test locally at localhost:4000/blog/post-title.html
# 6. Commit and deploy
```

### Deploy to Production
```bash
# On build Pi
cd ~/kevsrobots.com
git pull
cd ~/ClusteredPi/stacks/kevsrobots
docker build -t 192.168.2.1:5000/kevsrobots:latest .
docker push 192.168.2.1:5000/kevsrobots:latest

# On each backend Pi (dev01, dev02, dev03, dev04)
cd ~/ClusteredPi/stacks/kevsrobots
docker-compose pull
docker-compose down
docker-compose up -d

# Verify healthy
docker ps | grep kevsrobots
# Should show "healthy" after ~30 seconds
```

### Update Load Balancer
```bash
# On Pi running nginx load balancer
cd ~/ClusteredPi
git pull
cd stacks/nginx
docker-compose build
docker-compose down
docker-compose up -d
```

### Optimize Images
```bash
cd ~/kevsrobots.com
python3 optimize_images.py
# Optimizes all images in place
# Tracks optimized files in .optimized_images.json
# Safe to run multiple times
```

### Build Courses
```bash
cd ~/kevsrobots.com
python3 build.py
# Processes /source/ → /web/learn/
# Generates course pages and indexes
```

---

## Troubleshooting

### Footer Missing
- Check `_layouts/default.html` - footer must be BEFORE `</body>`
- Check `_includes/footer.html` exists
- Check `_includes/lazy_load.html` has closing `</script>` tag
- Hard refresh browser (Cmd+Shift+R)

### Container Unhealthy
- Check logs: `docker logs container-name`
- Verify healthcheck uses `127.0.0.1` not `localhost`
- Wait 30s for start-period grace period
- Check nginx is listening: `docker exec container netstat -tlnp`

### Timeout Errors
- Check nginx backend timeouts (60s)
- Check load balancer proxy timeouts (60s)
- Check image file sizes (run optimizer if >500KB)
- Check Cloudflare cache status

### Site Won't Build
- Check Jekyll logs: `docker logs stacks-jekyll-serve-1`
- Verify `_config.yml` syntax
- Check for invalid frontmatter in posts
- Look for unclosed HTML tags in layouts

---

## Git Commit Messages

Follow this format:
```
Brief description of change

- Detailed point 1
- Detailed point 2

Fixes: #issue-number (if applicable)

🤖 Generated with [Claude Code](https://claude.com/claude-code)

Co-Authored-By: Claude <noreply@anthropic.com>
```

---

## Performance Targets

- **Build time**: <2 minutes
- **Page load**: <2 seconds (with Cloudflare cache)
- **Container health**: 100% uptime
- **Image optimization**: <300KB per image
- **HTML size**: <150KB per page

---

## Security Notes

- **No credentials in repo** - Use environment variables
- **Health endpoint** - Returns plain 200 OK, no data
- **Cloudflare** - Handles SSL, DDoS protection, caching
- **Docker registry** - Local to Pi network (192.168.2.1:5000)
- **Rate limiting** - Handled by Cloudflare

---

## Known Issues

1. **HTML compression** - Currently disabled, breaks footer if too aggressive
2. **Page counter** - Depends on external service `page_count.kevsrobots.com:8005`
3. **IPv6** - Health checks must use IPv4 (127.0.0.1)
4. **ARM64** - Build on Pi or use `--platform linux/arm64` on Mac

---

## File Checklist Before Commit

- [ ] No debug code or console.logs
- [ ] All `<script>` tags closed properly
- [ ] Images optimized (<300KB)
- [ ] Footer inside `<body>` tag
- [ ] Frontmatter complete and valid
- [ ] Local build succeeds
- [ ] Browser console has no errors
- [ ] Links work (no 404s)

---

## External Dependencies

- **Cloudflare** - CDN, SSL, health checks
- **GitHub** - Source control, Docker cache busting
- **YouTube** - Video embeds
- **Font Awesome** - Icons
- **Bootstrap 5.3.3** - CSS framework
- **Highlight.js** - Code syntax highlighting

---

## Contact

- **Owner**: Kevin McAleer
- **Email**: kevinmcaleer@gmail.com
- **Website**: https://www.kevsrobots.com
- **GitHub**: https://github.com/kevinmcaleer/kevsrobots.com
