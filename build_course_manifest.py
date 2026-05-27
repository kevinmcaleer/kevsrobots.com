#!/usr/bin/env python3
"""
build_course_manifest.py

Scans /source/*/course.yml and emits /web/_data/course_lessons.json — an
ordered manifest of every course's lessons with their public URL paths.

This manifest is consumed client-side by /course-stats.html, which joins it
against live page_count data to render funnel/completion analytics.

Run alongside build.py whenever course content changes:
    python3 build.py
    python3 build_course_manifest.py
"""
import os
import glob
import json
import sys

try:
    import yaml
except ImportError:
    sys.exit("PyYAML required: pip install pyyaml")

ROOT = os.path.dirname(os.path.abspath(__file__))
SOURCE_DIR = os.path.join(ROOT, "source")
OUT_PATH = os.path.join(ROOT, "web", "_data", "course_lessons.json")


def lessons_from_course_yml(path):
    """Return ordered list of {'file': 'NN_name.md', 'title': str|None}."""
    with open(path) as f:
        data = yaml.safe_load(f)
    if not isinstance(data, list) or not data:
        return [], {}
    course = data[0]
    lessons = []
    for entry in course.get("content", []) or []:
        if not isinstance(entry, dict):
            continue
        sec = entry.get("section") or {}
        section_name = sec.get("name", "")
        for item in sec.get("content", []) or []:
            if isinstance(item, str) and item.endswith(".md"):
                lessons.append({"file": item, "section": section_name})
    meta = {
        "name": course.get("name", ""),
        "description": course.get("description", ""),
        "cover": course.get("cover", ""),
        "author": course.get("author", ""),
        "date_published": str(course.get("date_published", "")),
        "groups": course.get("groups", []) or [],
    }
    return lessons, meta


def build():
    out = []
    course_files = sorted(glob.glob(os.path.join(SOURCE_DIR, "*/course.yml")))
    for cf in course_files:
        slug = os.path.basename(os.path.dirname(cf))
        try:
            lessons, meta = lessons_from_course_yml(cf)
        except Exception as e:
            print(f"  ! skip {slug}: {e}", file=sys.stderr)
            continue
        if not lessons:
            continue
        # Public URL pattern: /learn/<slug>/<basename>.html
        for l in lessons:
            base = l["file"][:-3]  # strip .md
            l["url"] = f"/learn/{slug}/{base}.html"
        out.append({
            "slug": slug,
            "name": meta["name"] or slug,
            "description": meta["description"],
            "cover": meta["cover"],
            "author": meta["author"],
            "date_published": meta["date_published"],
            "groups": meta["groups"],
            "lessons": lessons,
        })

    os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
    with open(OUT_PATH, "w") as f:
        json.dump(out, f, indent=2)
    print(f"Wrote {len(out)} courses, {sum(len(c['lessons']) for c in out)} lessons -> {OUT_PATH}")


if __name__ == "__main__":
    build()
