#!/usr/bin/env python3
"""
KevsRobots MCP Server
Exposes site management scripts as MCP tools for Claude to call.
"""

import subprocess
import sys
from pathlib import Path

from mcp.server.fastmcp import FastMCP

# Root of the kevsrobots repo — adjust if this file moves
REPO_ROOT = Path(__file__).parent.parent.resolve()

mcp = FastMCP("kevsrobots")


def _run(cmd: list[str], cwd: Path | None = None) -> str:
    """Run a command and return combined stdout+stderr as a string."""
    result = subprocess.run(
        cmd,
        cwd=str(cwd or REPO_ROOT),
        capture_output=True,
        text=True,
    )
    output = []
    if result.stdout:
        output.append(result.stdout)
    if result.stderr:
        output.append(result.stderr)
    combined = "\n".join(output).strip()
    if result.returncode != 0:
        return f"ERROR (exit {result.returncode}):\n{combined}"
    return combined or "Done (no output)."


# ---------------------------------------------------------------------------
# Tools
# ---------------------------------------------------------------------------


@mcp.tool()
def rebuild_search_database() -> str:
    """Rebuild the site search database by running search_app/rebuild.sh.

    Deletes the old search.db index and re-indexes all content.
    """
    script = REPO_ROOT / "search_app" / "rebuild.sh"
    return _run(["bash", str(script)], cwd=REPO_ROOT / "search_app")


@mcp.tool()
def rebuild_courses() -> str:
    """Rebuild course pages from /source by running build.py.

    Processes all course.yml files and generates /web/learn content and
    updates /web/_data/courses.yml for Jekyll.
    """
    return _run([sys.executable, "build.py"])


@mcp.tool()
def rebuild_courses_database() -> str:
    """Rebuild the courses SQLite database by running course_app/build_database.py.

    Scans all courses and populates the courses.db used by the course filter API.
    """
    script = REPO_ROOT / "course_app" / "build_database.py"
    return _run([sys.executable, str(script)], cwd=REPO_ROOT / "course_app")


@mcp.tool()
def rebuild_posts_yaml() -> str:
    """Rebuild the posts YAML data file by running post_yaml_builder.py.

    Scans all Markdown blog posts and generates structured YAML metadata.
    """
    return _run([sys.executable, "post_yaml_builder.py"])


@mcp.tool()
def build_and_push_docker() -> str:
    """Build the Jekyll site Docker image and push to the local Pi registry.

    Runs:
      docker build . -t 192.168.2.1:5000/kevsrobots:latest
      docker push 192.168.2.1:5000/kevsrobots:latest

    Note: Docker must be running and the Pi registry must be reachable.
    This can take several minutes.
    """
    tag = "192.168.2.1:5000/kevsrobots:latest"
    build = _run(["docker", "build", ".", "-t", tag])
    if build.startswith("ERROR"):
        return build
    push = _run(["docker", "push", tag])
    return f"BUILD:\n{build}\n\nPUSH:\n{push}"


@mcp.tool()
def optimize_images() -> str:
    """Optimize all site images in-place using optimize_images.py.

    Compresses images to <300KB, tracks processed files in
    .optimization_cache.json. Safe to run multiple times.
    """
    return _run([sys.executable, "optimize_images.py"])


@mcp.tool()
def update_popular_videos() -> str:
    """Update popular YouTube video data by running popular_vids.py.

    Reads YouTube Studio analytics export and updates the site's
    popular videos data file.
    """
    return _run([sys.executable, "popular_vids.py"])


@mcp.tool()
def update_post_calendar() -> str:
    """Update the posts publication calendar by running post_calendar.py.

    Generates a visual calendar graphic showing which days have blog posts.
    """
    return _run([sys.executable, "post_calendar.py"])


@mcp.tool()
def update_year_in_review() -> str:
    """Update year-in-review charts and data by running charts.py.

    Generates analytics charts and statistics for the year-in-review section.
    """
    return _run([sys.executable, "charts.py"])


@mcp.tool()
def rebuild_all() -> str:
    """Run the full site rebuild sequence in order.

    Runs in sequence:
    1. rebuild_posts_yaml
    2. rebuild_courses
    3. rebuild_courses_database
    4. rebuild_search_database
    5. optimize_images
    6. update_popular_videos
    7. update_post_calendar
    8. update_year_in_review

    Does NOT build/push Docker — call build_and_push_docker separately.
    Returns a summary of each step.
    """
    steps = [
        ("Posts YAML", [sys.executable, "post_yaml_builder.py"], REPO_ROOT),
        ("Courses", [sys.executable, "build.py"], REPO_ROOT),
        ("Courses DB", [sys.executable, str(REPO_ROOT / "course_app" / "build_database.py")], REPO_ROOT / "course_app"),
        ("Search DB", ["bash", str(REPO_ROOT / "search_app" / "rebuild.sh")], REPO_ROOT / "search_app"),
        ("Optimize Images", [sys.executable, "optimize_images.py"], REPO_ROOT),
        ("Popular Videos", [sys.executable, "popular_vids.py"], REPO_ROOT),
        ("Post Calendar", [sys.executable, "post_calendar.py"], REPO_ROOT),
        ("Year in Review", [sys.executable, "charts.py"], REPO_ROOT),
    ]

    results = []
    for name, cmd, cwd in steps:
        output = _run(cmd, cwd=cwd)
        status = "✓" if not output.startswith("ERROR") else "✗"
        results.append(f"{status} {name}:\n{output}")

    return "\n\n".join(results)


if __name__ == "__main__":
    mcp.run()
