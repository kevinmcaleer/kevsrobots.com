# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Quick Reference

**Local Development:**
```bash
cd stacks && docker-compose up -d    # Start Jekyll server
# Visit http://localhost:4000
docker-compose down                   # Stop server
```

**Content Creation:**
```bash
python3 build.py                      # Build courses from /source
python3 optimize_images.py            # Optimize images
```

**Important Rules:**
- Blog posts: `YYYY-MM-DD-title.md` format in `/web/_posts/`
- Courses: Start with `00_intro.md`, each needs `course.yml`
- Groups: Categorize content in `/web/_groups/` with Font Awesome icons
- Images: `<300KB`, use `loading="lazy"`, optimize with script
- Docker: NEVER use `--no-cache` (Dockerfile has auto cache-busting)
- Jekyll: Footer MUST be inside `<body>`, all `<script>` tags must close properly

---

## Repository Structure

- **`/web`** - Jekyll site source files
  - `_posts/` - Blog posts (format: `YYYY-MM-DD-title.md`)
  - `_groups/` - Content category pages (electronics, micropython, robots, etc.)
  - `_layouts/` - Jekyll layout templates
  - `_includes/` - Reusable components (header, footer, etc.)
  - `_data/` - YAML data files (courses.yml, glossary.yml, navigation, etc.)
  - `_site/` - Generated static site (DO NOT edit directly)
  - `_config.yml` - Jekyll configuration
- **`/source`** - Course content (Markdown lessons with course.yml files)
- **`/stacks`** - Docker configurations for deployment
- **Python Scripts** (root directory):
  - `build.py` - Builds courses from `/source` to `/web/learn`
  - `optimize_images.py` - Optimizes images in-place, tracks with `.optimization_cache.json`
  - `build_search.py` - Builds search data from courses/glossary/posts
  - Other utility scripts for RSS, YouTube integration, analytics

---

## Content Architecture

### Content Types
The site has four main content types:

1. **Blog Posts** (`/web/_posts/`) - News, tutorials, project showcases
2. **Courses** (`/source/` → `/web/learn/`) - Structured learning content
3. **Groups** (`/web/_groups/`) - Category pages that organize content
4. **Data Files** (`/web/_data/`) - Structured data for navigation, courses, glossary

### Data Files Flow
```
source/*/course.yml  ──┐
                       ├─→ build.py ──→ web/_data/courses.yml ──→ Jekyll builds site
web/_posts/*.md      ──┘
```

Key data files:
- `courses.yml` - Generated from all course.yml files in /source
- `glossary.yml` - Terms and definitions for search/reference
- `navigation.yml` / `nav_*.yml` - Site navigation menus
- `youtube.yml` - YouTube video metadata for embeds

### Build Process
1. **Course Build** - `build.py` uses `course_builder` module to:
   - Read all `course.yml` files from `/source` subdirectories
   - Generate lesson HTML/MD files in `/web/learn/course-name/`
   - Update `/web/_data/courses.yml` with metadata for Jekyll
2. **Jekyll Build** - Reads `/web` directory, generates static site to `_site/`
3. **Search Index** - `build_search.py` combines courses, glossary, posts into searchable data
4. **Image Optimization** - `optimize_images.py` runs manually to compress images

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

### Groups (Content Categories)
- **Location** - `/web/_groups/` directory
- **File naming** - Use lowercase with hyphens: `electronics.md`, `micropython.md`
- **Frontmatter required** - title, description, date_published, layout (groups), cover, icon
- **Purpose** - Categorize and organize related content (courses, blog posts, projects)
- **Icon** - Use Font Awesome classes (e.g., `'fa-solid fa-bolt'` for electronics)

### Courses
- **Source** - Edit in `/source/course-name/` as Markdown
- **Build** - Run `python build.py` to generate `/web/learn/` content
- **Naming Convention** - Use `00_intro.md`, `01_lesson.md`, `02_lesson.md`, etc.
- **Structure** - Each course needs `course.yml` metadata file
- **Content Quality** - See "Course Content Guidelines" section below

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

## Common Development Commands

### Local Development
```bash
# Start local Jekyll server (from /stacks directory)
cd stacks
docker-compose up -d

# View site at http://localhost:4000
# LiveReload on port 35729 (auto-refresh on changes)

# Stop server
docker-compose down

# View logs
docker logs stacks-jekyll-serve-1
```

### Build and Test
```bash
# Build courses (processes /source → /web/learn)
python3 build.py

# Optimize images (safe to run multiple times)
python3 optimize_images.py

# Build search index
python3 build_search.py

# Check for broken links (if link_checker script exists)
python3 link_checker/linkchecker.py
```

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

### Add a New Content Group
```bash
# 1. Create group file
cd web/_groups
touch category-name.md

# 2. Add frontmatter with title, description, layout: groups, cover, icon
# 3. Add cover image to /web/assets/img/groups/
# 4. Test at localhost:4000/groups/category-name.html
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

## Course Content Guidelines

### Course Structure

Each course must have:

1. **`course.yml`** - Course metadata file
2. **Markdown lessons** - Numbered sequentially starting from `00_intro.md`
3. **`assets/`** folder - Course-specific images and resources

### course.yml Format

```yaml
- name: Course Name
  author: Kevin McAleer
  date_created: YYYY-MM-DD
  date_published: YYYY-MM-DD
  layout: course
  cover: assets/cover.jpg
  description: >-
    Brief description of what students will learn (1-2 sentences)
  groups:
    - relevant-category  # e.g., micropython, docker, robotics
  content:
  - section:
      name: Section Name
      content:
      - 00_intro.md
      - 01_lesson_name.md
      - 02_lesson_name.md
  - section:
      name: Another Section
      content:
      - 03_lesson_name.md
```

### Lesson Frontmatter (Required)

```yaml
---
title: Lesson Title
description: Brief description for SEO and previews
layout: lesson
type: page
cover: /learn/course-name/assets/image.jpg  # Optional
---
```

### Lesson File Naming

- **Always start with `00_intro.md`** for course introduction
- **Number sequentially**: `01_`, `02_`, `03_`, etc.
- **Use descriptive names**: `01_what-is-docker.md` not `01_lesson.md`
- **Use hyphens** for spaces: `05_hello-world.md`

### Content Quality Standards

Based on course analysis, all courses should follow these guidelines:

#### 1. **Consistent Introduction Structure**

Every course `00_intro.md` must include:

```markdown
## Overview
Brief paragraph about what the course covers

## Course Content
Bullet list of topics covered:
- Topic 1
- Topic 2
- Topic 3

## Key Results
What students will be able to do after completing:
- Skill 1
- Skill 2

## What you'll need
List of required hardware/software:
- Item 1
- Item 2

## How the course works
Explain formatting conventions, code examples, notes
```

#### 2. **Real-World Examples Required**

**BAD** - Reference table only:
```markdown
| Operator | Symbol | Description |
|----------|--------|-------------|
| Addition | `+`    | Adds values |
```

**GOOD** - Real-world context:
```markdown
### Addition Operator (`+`)

**What it does:** Adds two numbers together

**Real-world robot example:**
```python
# Calculate total motor speed
left_motor_speed = 50
right_motor_speed = 30
total_speed = left_motor_speed + right_motor_speed
print(f"Total: {total_speed}")  # Output: 80
```

**Common mistake:**
```python
speed = "50"  # String, not number!
total = speed + 10  # TypeError!
```
```

#### 3. **Progressive Learning**

- **Start simple** - Basic concepts first
- **Build complexity** - Each lesson adds one new concept
- **Practical application** - Show how to use it, not just what it is
- **Build toward a project** - Each course should culminate in a working project

#### 4. **Interactive Elements**

Add to each lesson:

```markdown
## Try it Yourself

1. Modify the code to...
2. Experiment with...
3. Challenge: Can you...

## Common Issues

- **Problem**: Error message or issue
- **Solution**: How to fix it
- **Why**: Explanation

- **Problem**: Another error or issue
- **Solution**: Steps to resolve it
- **Why**: Explanation of root cause
```

#### 5. **Course Progression Indicators**

Add breadcrumbs to lessons:

```markdown
> **Course Progress**: Lesson 8 of 20
>
> **Previous**: [Variables](/learn/course/07_variables.html) |
> **Next**: [Functions](/learn/course/09_functions.html)
```

#### 6. **Learning Paths**

Courses should link to related courses:

```markdown
## Prerequisites

This course assumes you've completed:
- [Introduction to MicroPython](/learn/micropython/)

## What's Next

After this course, try:
- [MicroPython GPIO](/learn/micropython_gpio/)
- [Building Your First Robot](/learn/smars/)
```

### Content Length Guidelines

- **Intro lesson**: 300-500 words
- **Concept lessons**: 400-800 words
- **Project lessons**: 800-1500 words
- **Total course**: 10-20 lessons minimum

### Tone and Style

- **Friendly and encouraging** - "Let's build..." not "You must..."
- **Maker-focused** - Relate to robots, hardware, projects
- **Practical** - Show working code, not just theory
- **Conversational** - Write like you're teaching a friend
- **Visual** - Use images, diagrams, code examples

### Things to Avoid

❌ Long reference tables without context
❌ Theory-heavy content without examples
❌ Assuming prior knowledge
❌ Skipping troubleshooting sections
❌ No clear end goal or project
❌ Inconsistent lesson naming (00_intro vs 01_intro)

### Course Review Checklist

Before publishing a course:

- [ ] All lessons start with `00_intro.md`
- [ ] `course.yml` complete with all required fields
- [ ] Each lesson has proper frontmatter
- [ ] Code examples are tested and working
- [ ] Images optimized (<300KB)
- [ ] Real-world examples for each concept
- [ ] "Try it Yourself" sections included
- [ ] Troubleshooting sections for complex topics
- [ ] Clear progression from beginner to project
- [ ] Links to prerequisite/next courses

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
