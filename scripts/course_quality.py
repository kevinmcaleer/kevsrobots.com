#!/usr/bin/env python3
"""Score courses against the course quality checklist (epic #655).

    python3 scripts/course_quality.py                 # every course, weakest first
    python3 scripts/course_quality.py source/python   # one course, with its gaps
    python3 scripts/course_quality.py --write         # also write web/_data/course_quality.yml
    python3 scripts/course_quality.py --strict PATHS  # exit 1 on placeholder lessons,
                                                      # missing images or emoji headings
    python3 scripts/course_quality.py --min 80 PATHS  # exit 1 if a course scores under 80

100 points over eight areas:

    video       15  video in the intro (or lesson 1), more videos in later lessons
    interactive 20  a quiz and a Try it Yourself in most lessons, plus something to do
                    on the page: runnable Python, a widget or diagram, or step photos
    objectives  10  outcomes / prerequisites / you_need in course.yml, and each lesson
                    opens by saying what the learner will be able to do
    teachers     5  a Teachers note in the intro, and on lessons with a classroom catch
    content     20  no stub lessons, Common Issues, visuals, intro and summary structure
    accuracy    15  code check at 0 errors, no missing images or placeholder lessons,
                    no dated commands
    pathway      5  in a learning pathway, level and groups set
    style       10  no emoji headings, tables classed, lesson banners, 4:3 cover,
                    descriptions, starts at 00_

Bands: A 80+, B 65 to 79, C 45 to 64, D under 45. A course is done at 80 or more.

The score measures what is on the page. It can't tell whether the prose is true:
facts still need checking by a person against the real tool and the video.
"""

import argparse
import glob
import os
import re
import statistics
import subprocess
import sys

import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(REPO, "source")

EMOJI = re.compile("[\U0001F000-\U0001FAFF☀-➿▶]")
OBJECTIVE = re.compile(
    r"learning objectives|what you(’|')?ll learn|you(’|')?ll learn|you will learn|"
    r"by the end of this (lesson|course)|in this lesson|key results|you will be able to|"
    r"you(’|')?ll be able to",
    re.I,
)
TRY_IT = re.compile(r"try it yourself|challenge|exercise|your turn|have a go", re.I)
ISSUES = re.compile(r"common (issues|problems|mistakes)|troubleshoot", re.I)
TEACHER = re.compile(r"\bteachers?\b|classroom|educator", re.I)
STALE = re.compile(
    r"docker-compose |version: .3|\bBuster\b|\bBullseye\b|\bStretch\b|apt-key|sudo pip|"
    r"\bimport utime|\bimport ujson|\bimport urequests|\bpicamera\b|python2|ROS Noetic|"
    r"\bMelodic\b|\bFoxy\b|\bGalactic\b|Ubuntu 1[68]|Ubuntu 20|rshell|Pi 3\b|Python 3\.[5-8]\b"
)
AREAS = [
    ("video", "Video front and centre", 15),
    ("interactive", "Interactivity", 20),
    ("objectives", "Learning objectives", 10),
    ("teachers", "Notes for teachers", 5),
    ("content", "Content quality", 20),
    ("accuracy", "Accuracy and code", 15),
    ("pathway", "Pathway and navigation", 5),
    ("style", "House style", 10),
]


def band(total):
    return "A" if total >= 80 else "B" if total >= 65 else "C" if total >= 45 else "D"


def pathways():
    """Course name -> list of pathway names."""
    found = {}
    for path in glob.glob(os.path.join(REPO, "web", "_learning_pathways", "*.md")):
        match = re.match(r"---\n(.*?)\n---", open(path, encoding="utf-8").read(), re.S)
        if not match:
            continue
        data = yaml.safe_load(match.group(1)) or {}
        names = list(data.get("courses") or [])
        for stage in (data.get("stages") or {}).values():
            names += stage or []
        for name in names:
            found.setdefault(name, []).append(data.get("name") or os.path.basename(path))
    return found


def read_lesson(slug, filename):
    path = os.path.join(SRC, slug, filename)
    if not os.path.exists(path):
        return None
    text = open(path, encoding="utf-8", errors="replace").read()
    front = {}
    body = text
    match = re.match(r"---\n(.*?)\n---\n?", text, re.S)
    if match:
        try:
            front = yaml.safe_load(match.group(1)) or {}
        except yaml.YAMLError:
            front = {}
        body = text[match.end():]
    body = re.sub(r"<!--(?! *code-check).*?-->", "", body, flags=re.S)
    prose = re.sub(r"```.*?```", "", body, flags=re.S)
    plain = re.sub(r"\{%.*?%\}", "", prose, flags=re.S)
    words = len(re.findall(r"[A-Za-z0-9']+", re.sub(r"<[^>]+>|\(/[^)]*\)", "", plain)))
    fences = re.findall(r"```([a-zA-Z0-9_+-]*)\n(.*?)```(\n\{: *\.no-snakie *\})?", body, re.S)
    python = [f for f in fences if f[0].lower() in ("python", "py", "micropython")]
    heads = re.findall(r"^#{1,4} +(.*)$", prose, re.M)
    images = re.findall(r"!\[[^\]]*\]\(([^)\s]+)", re.sub(r"`[^`\n]*`", "", prose))
    images += re.findall(r'<img[^>]+src="([^"]+)"', prose)
    images = [i for i in images if "page.cover" not in i]
    missing = []
    for image in images:
        if image.startswith("http") or "{{" in image:
            continue
        if image.startswith(f"/learn/{slug}/"):
            target = os.path.join(SRC, slug, image[len(f"/learn/{slug}/"):])
        elif image.startswith("/"):
            target = os.path.join(REPO, "web", image.lstrip("/"))
        else:
            target = os.path.join(SRC, slug, image)
        if not os.path.exists(target):
            missing.append(image)
    is_video = front.get("type") == "video"
    return {
        "file": filename,
        "words": words,
        "video": len(re.findall(r"youtubeplayer|youtube\.com/embed|youtu\.be/|youtube\.com/watch", body))
        + (1 if front.get("video_id") or is_video else 0),
        "video_lesson": is_video,
        "quiz": len(re.findall(r"\{% *quiz", body)),
        "snakie": len(re.findall(r"include snakie\.html", body)),
        "code": len(fences),
        "python": len(python),
        "runnable": len([f for f in python if not f[2]]),
        "widget": len(re.findall(r"<script|<iframe|<canvas|<input|<details|stl_viewer|include explainer|include uno_q", body)),
        "mermaid": len(re.findall(r"```mermaid|\{% *mermaid", body)),
        "images": len(images),
        "missing": missing,
        "try_it": bool(TRY_IT.search(prose)),
        "issues": bool(ISSUES.search(prose)),
        "objective": bool(OBJECTIVE.search(prose)),
        "teacher": bool(TEACHER.search(prose)),
        "emoji": sum(1 for h in heads if EMOJI.search(h)) + (1 if EMOJI.search(str(front.get("title", ""))) else 0),
        "tables": len(re.findall(r"^\|? *:?-{3,}.*\|.*$", prose, re.M)),
        "tables_classed": len(re.findall(r'\{: *class="table', body)),
        "banner": "banners/" in str(front.get("cover", "")),
        "has_cover": bool(re.search(r'class="cover"', body)),
        "description": bool(front.get("description")),
        "heads": heads,
        "stale": sorted(set(m.lower().strip() for m in STALE.findall(body))),
    }


def code_errors(slug):
    """Errors from scripts/check_course_code.py, or None if it can't run."""
    try:
        run = subprocess.run(
            [sys.executable, os.path.join(REPO, "scripts", "check_course_code.py"), "--errors-only",
             os.path.join(SRC, slug)],
            capture_output=True, text=True, timeout=600,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    match = re.search(r"(\d+) errors", run.stdout + run.stderr)
    return int(match.group(1)) if match else None


def score_course(slug, in_pathways, run_code_check=True):
    manifest = yaml.safe_load(open(os.path.join(SRC, slug, "course.yml"), encoding="utf-8"))[0]
    files = [f for section in manifest.get("content", []) for f in section["section"]["content"]]
    lessons = [l for l in (read_lesson(slug, f) for f in files) if l]
    if not lessons:
        return None
    n = len(lessons)
    content = [l for l in lessons if not l["video_lesson"]] or lessons
    body = content[1:] or content  # the intro is left out of per-lesson ratios
    nb = len(body)

    def count(key, group=None):
        return sum(1 for l in (body if group is None else group) if l[key])

    python_blocks = sum(l["python"] for l in lessons)
    code_blocks = sum(l["code"] for l in lessons)
    kind = "python" if python_blocks >= 5 else "code" if code_blocks >= 5 else "hands-on"
    gaps = []

    # Video (15)
    with_video = [i for i, l in enumerate(lessons) if l["video"]]
    video = 0.0
    if with_video:
        video = 9 if with_video[0] <= 1 else 5
        if with_video[0] > 1:
            gaps.append("video is not in the intro")
    else:
        gaps.append("no video")
    video += 6 * min(1, len(with_video) / (0.25 * n))

    # Interactivity (20)
    quiz = 8 * min(1, count("quiz") / (0.8 * nb))
    try_it = 6 * min(1, count("try_it") / (0.7 * nb))
    if kind == "python":
        hands = 6 * min(1, sum(1 for l in body if l["runnable"] or l["snakie"]) / (0.5 * nb))
    elif kind == "code":
        hands = 3 * min(1, sum(1 for l in body if l["code"]) / (0.5 * nb))
        hands += 3 * min(1, sum(1 for l in body if l["widget"] or l["mermaid"]) / (0.5 * nb))
    else:
        hands = 6 * min(1, (sum(l["images"] + l["mermaid"] for l in body) / nb) / 2)
    interactive = quiz + try_it + hands
    no_quiz = [l["file"] for l in body if not l["quiz"]]
    if no_quiz:
        gaps.append(f"no quiz in {len(no_quiz)} of {nb} lessons: " + ", ".join(no_quiz))
    no_try = [l["file"] for l in body if not l["try_it"]]
    if len(no_try) > 0.3 * nb:
        gaps.append(f"no Try it Yourself in {len(no_try)} lessons: " + ", ".join(no_try))

    # Objectives (10)
    intro = lessons[0]
    intro_heads = " ".join(intro["heads"]).lower()
    objectives = 3 * bool(manifest.get("outcomes"))
    objectives += 1 * bool(manifest.get("prerequisites") or "prereq" in intro_heads or "before you" in intro_heads)
    objectives += 1 * bool(manifest.get("you_need") or "need" in intro_heads)
    objectives += 5 * min(1, count("objective") / (0.7 * nb))
    if not manifest.get("outcomes"):
        gaps.append("no outcomes in course.yml")
    no_objective = [l["file"] for l in body if not l["objective"]]
    if len(no_objective) > 0.3 * nb:
        gaps.append(f"no opening objective in {len(no_objective)} lessons: " + ", ".join(no_objective))

    # Teachers (5)
    teachers = 3 * bool(intro["teacher"]) + 2 * min(1, sum(1 for l in body if l["teacher"]) / 2)
    if not intro["teacher"]:
        gaps.append("no Teachers note in the intro")
        if any(l["teacher"] for l in content):
            teachers = max(teachers, 1)

    # Content (20)
    stubs = [l["file"] for l in content if l["words"] < 120]
    median = statistics.median(l["words"] for l in content)
    depth = 8 * (1 - min(1, 2 * len(stubs) / len(content))) * min(1, median / 350)
    issues = 4 * min(1, count("issues") / (0.5 * nb))
    visuals = 4 * min(1, sum(l["images"] + l["mermaid"] for l in body) / nb)
    last = lessons[-1]
    structure = 0.5 * intro["file"].startswith("00")
    structure += 1.0 * ("overview" in intro_heads or "course content" in intro_heads or "what you" in intro_heads)
    structure += 0.5 * ("need" in intro_heads)
    structure += 1.0 * bool(re.search("summary|conclusion|wrap|next|recap|review", last["file"]))
    structure += 1.0 * (last["words"] >= 150)
    content_score = depth + issues + visuals + structure
    if stubs:
        gaps.append(f"{len(stubs)} lessons under 120 words: " + ", ".join(stubs))
    no_visual = [l["file"] for l in body if not (l["images"] + l["mermaid"])]
    if len(no_visual) > 0.5 * nb:
        gaps.append(f"no picture or diagram in {len(no_visual)} lessons")
    if count("issues") < 0.5 * nb:
        gaps.append(f"Common Issues in only {count('issues')} of {nb} lessons")

    # Accuracy (15)
    errors = code_errors(slug) if run_code_check else 0
    missing = [(l["file"], image) for l in lessons for image in l["missing"]]
    placeholders = [l["file"] for l in lessons if l["words"] < 30 and not l["video"]]
    stale = sorted(set(s for l in lessons for s in l["stale"]))
    accuracy = (8 if not errors else 4 if errors <= 2 else 0)
    accuracy += max(0, 4 - len(missing) - 2 * len(placeholders)) + max(0, 3 - len(stale))
    if errors:
        gaps.append(f"{errors} code-check errors")
    if errors is None:
        gaps.append("code check could not run (build MicroPython with scripts/get_micropython.sh)")
    if missing:
        gaps.append("missing images: " + ", ".join(f"{f} -> {i}" for f, i in missing))
    if placeholders:
        gaps.append("placeholder lessons: " + ", ".join(placeholders))
    if stale:
        gaps.append("dated commands or versions: " + ", ".join(stale))

    # Pathway (5)
    in_path = in_pathways.get(manifest["name"], [])
    pathway = 3 * bool(in_path) + 1 * bool(manifest.get("level")) + 1 * bool(manifest.get("groups"))
    if not in_path:
        gaps.append("not in a learning pathway")
    if not manifest.get("level"):
        gaps.append("no level in course.yml")

    # Style (10)
    emoji = sum(l["emoji"] for l in lessons)
    tables = sum(l["tables"] for l in lessons)
    classed = sum(l["tables_classed"] for l in lessons)
    headers = sum(1 for l in lessons if l["banner"] or l["has_cover"])
    cover_ok = False
    cover = os.path.join(SRC, slug, str(manifest.get("cover", "")).lstrip("/"))
    try:
        from PIL import Image
        width, height = Image.open(cover).size
        cover_ok = abs(width / height - 4 / 3) < 0.02
    except Exception:
        cover_ok = False
    style = (3 if emoji == 0 else 1.5 if emoji <= 3 else 0)
    style += 2 * (1 if tables == 0 else min(1, classed / tables))
    style += 2 * headers / n + 1 * cover_ok + 1 * (count("description", lessons) == n)
    style += 1 * intro["file"].startswith("00")
    if emoji:
        gaps.append(f"{emoji} emoji headings")
    if tables > classed:
        gaps.append(f"{tables - classed} unstyled tables")
    if headers < n:
        gaps.append(f"{n - headers} lessons with no header banner")
    if not cover_ok:
        gaps.append("cover is missing or not 4:3")
    if not intro["file"].startswith("00"):
        gaps.append(f"first lesson is {intro['file']}, not 00_intro.md")

    areas = {
        "video": video, "interactive": interactive, "objectives": objectives, "teachers": teachers,
        "content": content_score, "accuracy": accuracy, "pathway": pathway, "style": style,
    }
    total = round(sum(areas.values()))
    hard = []
    if placeholders:
        hard.append("placeholder lessons: " + ", ".join(placeholders))
    if missing:
        hard.append("missing images: " + ", ".join(f"{f} -> {i}" for f, i in missing))
    if emoji:
        hard.append(f"{emoji} emoji in titles or headings")
    return {
        "slug": slug, "name": manifest["name"], "lessons": n, "kind": kind, "score": total,
        "band": band(total), "areas": {k: round(v, 1) for k, v in areas.items()},
        "gaps": gaps, "hard": hard,
    }


def slugs_from(paths):
    if not paths:
        return sorted(s for s in os.listdir(SRC) if os.path.exists(os.path.join(SRC, s, "course.yml")))
    found = []
    for path in paths:
        parts = os.path.normpath(path).split(os.sep)
        slug = parts[parts.index("source") + 1] if "source" in parts and len(parts) > parts.index("source") + 1 else parts[-1]
        if os.path.exists(os.path.join(SRC, slug, "course.yml")) and slug not in found:
            found.append(slug)
    return found


# Courses with hard failures that predate the strict check. Each is tracked by
# the issue listed, and its entry is removed when that course is reworked.
KNOWN_DEBT = {"learning_platform": 706}


def main():
    parser = argparse.ArgumentParser(description="Score courses against the course quality checklist.")
    parser.add_argument("paths", nargs="*", help="course folders or lesson files (default: every course)")
    parser.add_argument("--write", action="store_true", help="write web/_data/course_quality.yml")
    parser.add_argument("--strict", action="store_true", help="fail on placeholder lessons, missing images, emoji headings")
    parser.add_argument("--min", type=int, default=0, help="fail if a course scores under this")
    parser.add_argument("--no-code-check", action="store_true", help="skip scripts/check_course_code.py")
    parser.add_argument("--gaps", action="store_true", help="list each course's gaps")
    args = parser.parse_args()

    in_pathways = pathways()
    slugs = slugs_from(args.paths)
    results = [r for r in (score_course(s, in_pathways, not args.no_code_check) for s in slugs) if r]
    results.sort(key=lambda r: (r["score"], r["slug"]))
    show_gaps = args.gaps or len(results) <= 3

    print(f"{'score':>5} band  " + "  ".join(f"{key[:5]:>5}" for key, _label, _max in AREAS) + "  course")
    for r in results:
        print(f"{r['score']:>5}    {r['band']}  " + "  ".join(f"{r['areas'][key]:>5g}" for key, _l, _m in AREAS) + f"  {r['slug']}")
        if show_gaps:
            for gap in r["gaps"]:
                print(f"        - {gap}")
    if len(results) > 1:
        counts = {b: sum(1 for r in results if r["band"] == b) for b in "ABCD"}
        print(f"\n{len(results)} courses: " + ", ".join(f"{counts[b]} {b}" for b in "ABCD")
              + f", average {sum(r['score'] for r in results) / len(results):.0f}")

    if args.write:
        data = {r["slug"]: {"score": r["score"], "band": r["band"], "areas": r["areas"], "gaps": r["gaps"]}
                for r in sorted(results, key=lambda r: r["slug"])}
        out = os.path.join(REPO, "web", "_data", "course_quality.yml")
        with open(out, "w", encoding="utf-8") as handle:
            handle.write("# Written by scripts/course_quality.py --write. Do not edit by hand.\n")
            yaml.safe_dump(data, handle, sort_keys=True, allow_unicode=True, width=1000)
        print(f"wrote {os.path.relpath(out, REPO)}")

    failed = False
    for r in results:
        if args.strict and r["hard"]:
            issue = KNOWN_DEBT.get(r["slug"])
            for item in r["hard"]:
                if issue:
                    print(f"::warning::{r['slug']}: {item} (known, tracked in #{issue})")
                else:
                    failed = True
                    print(f"::error::{r['slug']}: {item}")
        if args.min and r["score"] < args.min:
            failed = True
            print(f"::error::{r['slug']} scores {r['score']}, under {args.min}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
