# Course Builder - course class
# Kevin McAleer
# October 2022

import os
import re
import shutil
import yaml
from math import ceil
from pathlib import Path


QUIZ_BLOCK = re.compile(r"{%-?\s*quiz\s*-?%}(.*?){%-?\s*endquiz\s*-?%}", re.S)
YOUTUBE_EMBED = re.compile(r"{%-?\s*include\s+youtubeplayer\.html\s+id=[\"']?([\w-]{6,})")


class Course:

    def __init__(
        self,
        name="",
        description="",
        author="",
        date_created="",
        date_published="",
        content=None,
        layout="",
        type="page",
        links=None,
        groups=None,
        output="nav.html",
        output_folder="not_specified",
        course_folder="",
        duration=0,
    ):
        self.name = name
        self.description = description
        self.author = author
        self.date_created = date_created
        self.date_published = date_published
        self.content = content or []
        self.layout = layout
        self.type = type
        self.links = links or []
        self.groups = groups or []
        self.output = output
        self.output_folder = output_folder
        self.course_folder = course_folder
        self.level = "beginner"
        self.cover = ""
        self.link = ""
        self.duration = duration
        self._course_manifest = None
        self._lesson_cache = {}
        self._navigation_yaml = None

    def _load_manifest(self):
        """Load and cache the course.yml manifest."""
        if self._course_manifest is not None:
            return self._course_manifest

        manifest_path = Path(self.course_folder) / "course.yml"
        try:
            with open(manifest_path, "r") as stream:
                data = yaml.safe_load(stream)
        except (yaml.YAMLError, OSError) as exc:
            print(f"Error loading course manifest {manifest_path}: {exc}")
            return None

        if not data:
            print(f"Error loading course manifest: {manifest_path} is empty")
            return None

        self._course_manifest = data[0]
        return self._course_manifest

    def _read_lesson_file(self, filename):
        """Read and cache a lesson file, returning (front_matter_dict, body_text, raw_content)."""
        if filename in self._lesson_cache:
            return self._lesson_cache[filename]

        path = Path(self.course_folder) / filename
        try:
            raw_content = path.read_text()
        except OSError as exc:
            print(f"Warning: could not read lesson file {path}: {exc}")
            self._lesson_cache[filename] = ({}, "", "")
            return {}, "", ""

        front_matter = {}
        body_text = raw_content
        parts = raw_content.split("---", 2)
        if len(parts) >= 3:
            try:
                front_matter = yaml.safe_load(parts[1]) or {}
            except yaml.YAMLError as exc:
                print(f"Warning: invalid front matter in {path}: {exc}")
                front_matter = {}
            body_text = parts[2]

        result = (front_matter, body_text, raw_content)
        self._lesson_cache[filename] = result
        return result

    def _get_lesson_duration(self, filename):
        """Get the duration of a lesson in minutes."""
        front_matter, body_text, _ = self._read_lesson_file(filename)

        if isinstance(front_matter, dict) and "duration" in front_matter:
            try:
                return int(front_matter["duration"])
            except (ValueError, TypeError):
                pass

        words = len(body_text.split())
        return ceil(words / 200)

    def _get_quiz_questions(self, filename):
        """Count the quiz questions (`?` lines inside {% quiz %} blocks) in a lesson."""
        _, body_text, _ = self._read_lesson_file(filename)
        count = 0
        for block in QUIZ_BLOCK.findall(body_text):
            count += sum(1 for line in block.splitlines() if line.strip().startswith("?"))
        return count

    @property
    def video(self):
        """YouTube id of the course's video, or None.

        `video:` in course.yml wins. Otherwise it is the first video embedded
        with {% include youtubeplayer.html id="..." %}, looking through the
        lessons in order, so the intro's video is found first.
        """
        manifest_video = (self._course_manifest or {}).get("video")
        if manifest_video:
            return str(manifest_video)
        for item in self.lessons:
            _, body_text, _ = self._read_lesson_file(item)
            match = YOUTUBE_EMBED.search(body_text)
            if match:
                return match.group(1)
        return None

    @property
    def quiz_total(self):
        """Total quiz questions across the whole course."""
        return sum(self._get_quiz_questions(item) for item in self.lessons)

    def calculate_duration(self):
        """Calculate total course duration from all lessons."""
        duration = 0
        for item in self.lessons:
            duration += self._get_lesson_duration(item)
        self.duration = duration
        return duration

    def read_course(self, course_folder):
        """Read a course manifest and populate this Course object."""
        self.course_folder = course_folder
        self._course_manifest = None
        self._lesson_cache = {}
        self._navigation_yaml = None

        manifest = self._load_manifest()
        if manifest is None:
            return None

        try:
            self.author = manifest["author"]
            self.name = manifest["name"]
            self.description = manifest["description"]
            self.groups = manifest["groups"]
            self.date_created = manifest["date_created"]
            self.date_published = manifest["date_published"]
            self.content = manifest["content"]
            self.level = manifest.get("level", "beginner")
            # Projects to suggest at the end of the course ("Build something
            # next"), as site URLs. Nibsy shows these first (#632).
            self.projects = [str(p) for p in manifest.get("projects") or []]
            # Optional, for the course overview page (#635): what you will
            # be able to do, what you should know first, what to have ready.
            self.outcomes = [str(o) for o in manifest.get("outcomes") or []]
            self.prerequisites = [str(o) for o in manifest.get("prerequisites") or []]
            self.you_need = [str(o) for o in manifest.get("you_need") or []]
            cover_folder = self.course_folder.replace("source", "/learn")
            self.cover = cover_folder + "/" + manifest["cover"]
            self.cover = self.cover.replace("//learn", "/learn")
        except KeyError as exc:
            print(f"Error reading course manifest: {exc}, course name is: {self.name}")
            return None

        self.calculate_duration()
        return self

    def build_index(self):
        """Write index.md, the course overview page (#635).

        It used to be a copy of the first lesson. Now it is its own page
        (layout: course_overview) with the course's details, what you will
        learn, what you need, and the syllabus; the body is empty.
        """
        if not self.lessons:
            return
        first = self.lessons[0].replace(".md", ".html")
        generated = {
            "layout": "course_overview",
            "title": self.name,
            "author": self.author,
            "cover": self.cover,
            "date": self.date_published,
            "description": self.description,
            "level": self.level,
            "course_duration": self.duration_str,
            "lesson_count": self.no_of_lessons,
            "first_lesson": first,
        }
        if self.quiz_total:
            generated["quiz_total"] = self.quiz_total
        if getattr(self, "video", None):
            generated["video"] = self.video
        for key in ("outcomes", "prerequisites", "you_need"):
            if getattr(self, key, None):
                generated[key] = getattr(self, key)
        page = (
            "---\n"
            + yaml.safe_dump(generated, sort_keys=False, allow_unicode=True)
            + "navigation:\n"
            + self.build_navigation()
            + "---\n"
        )
        (Path(self.output_folder) / "index.md").write_text(page)

    def create_output(self, output_folder=None):
        """Create the output directory for the course."""
        print(f"Building course: {self.name}")
        if output_folder:
            self.output_folder = output_folder
        os.makedirs(self.output_folder, exist_ok=True)

    @property
    def lessons(self) -> list:
        """Return a flat list of all lesson filenames."""
        lesson_list = []
        for section in self.content:
            for item in section["section"]["content"]:
                lesson_list.append(item)
        return lesson_list

    def next_lesson(self, lesson_number):
        """Return the filename of the next lesson (as .html), or None."""
        if lesson_number < self.no_of_lessons:
            return self.lessons[lesson_number].replace(".md", ".html")
        return None

    def previous_lesson(self, lesson_number):
        """Return the filename of the previous lesson (as .html), or None."""
        if lesson_number > 1:
            return self.lessons[lesson_number - 2].replace(".md", ".html")
        return None

    @property
    def no_of_lessons(self):
        """Count the total number of lessons across all sections."""
        count = 0
        for section in self.content:
            count += len(section["section"]["content"])
        return count

    def build_navigation(self):
        """Build the navigation YAML structure. Cached per course."""
        if self._navigation_yaml is not None:
            return self._navigation_yaml

        manifest = self._load_manifest()
        if manifest is None:
            self._navigation_yaml = ""
            return ""

        nav = [{"name": manifest["name"]}]
        sections = []

        for section in manifest["content"]:
            section_data = section["section"]
            item_records = []
            for item in section_data["content"]:
                front_matter, _, _ = self._read_lesson_file(item)
                title = item
                if isinstance(front_matter, dict) and "title" in front_matter:
                    title = front_matter["title"]
                link = item.replace(".md", ".html")
                item_records.append({"name": title, "link": link})
            sections.append({"section": section_data["name"], "content": item_records})

        nav.append({"content": sections})

        self._navigation_yaml = yaml.dump(nav, sort_keys=False)
        return self._navigation_yaml

    def update_front_matter(self, lesson_file, front_matter, nav_yaml):
        """Merge generated front matter with the lesson's existing front matter."""
        lesson_list = lesson_file.split("---", 2)

        existing_fm = {}
        if len(lesson_list) >= 2:
            try:
                existing_fm = yaml.safe_load(lesson_list[1]) or {}
            except yaml.YAMLError as exc:
                print(exc)
                existing_fm = {}

        content = lesson_list[2] if len(lesson_list) >= 3 else lesson_file

        front_matter_list = front_matter.split("---", 2)
        generated_fm = {}
        if len(front_matter_list) >= 2:
            try:
                generated_fm = yaml.safe_load(front_matter_list[1]) or {}
            except yaml.YAMLError as exc:
                print(exc)
                generated_fm = {}

        merged = {}
        merged.update(generated_fm)
        if isinstance(existing_fm, dict):
            merged.update(existing_fm)

        merged_yaml = yaml.dump(merged, sort_keys=False)

        page = (
            "---\n"
            + merged_yaml
            + "navigation:\n"
            + nav_yaml
            + "---\n"
            + content
        )
        return page

    def copy_assets(self):
        """Copy the assets folder to the output folder."""
        src = Path(self.course_folder) / "assets"
        dst = Path(self.output_folder) / "assets"

        if not src.exists():
            return

        if dst.exists():
            shutil.rmtree(str(dst))
        shutil.copytree(str(src), str(dst))

    def build(self):
        """Build the front matter for each content page and output to the build folder."""
        self.create_output(self.output_folder)

        total_lessons = self.no_of_lessons
        if total_lessons == 0:
            print("no content found")
            return self.duration

        page_percent = 100 // total_lessons

        # Build navigation once for the entire course
        nav_yaml = self.build_navigation()

        page_count = 1
        quiz_total = self.quiz_total

        for section in self.content:
            section_data = section["section"]

            for item in section_data["content"]:
                if page_count == 1:
                    self.link = item.replace(".md", ".html")

                item_percentage = min(page_percent * page_count, 100)
                if page_count == total_lessons:
                    item_percentage = 100

                next_l = self.next_lesson(page_count)
                previous_l = self.previous_lesson(page_count)

                if page_count < total_lessons:
                    page_count += 1

                # Get lesson duration from cache
                lesson_duration = self._get_lesson_duration(item)

                # Build as a dict and let yaml quote the values: a description
                # containing ": " (e.g. "Arduino C++: drive it") is invalid
                # YAML when pasted in raw, and the whole block was dropped.
                generated = {
                    "layout": self.layout,
                    "title": item,
                    "author": self.author,
                    "type": self.type,
                    "cover": self.cover,
                    "date": self.date_published,
                }
                if previous_l is not None:
                    generated["previous"] = previous_l
                if next_l is not None:
                    generated["next"] = next_l
                generated["description"] = self.description
                generated["percent"] = item_percentage
                generated["duration"] = lesson_duration
                if quiz_total:
                    generated["quiz_total"] = quiz_total
                front_matter = (
                    "---\n"
                    + yaml.safe_dump(generated, sort_keys=False, allow_unicode=True)
                    + "---\n"
                )

                # Read lesson file content (from cache)
                _, _, raw_content = self._read_lesson_file(item)
                if not raw_content:
                    print(f"Warning: skipping empty/missing lesson {item}")
                    continue

                page = self.update_front_matter(
                    lesson_file=raw_content,
                    front_matter=front_matter,
                    nav_yaml=nav_yaml,
                )

                output_path = Path(self.output_folder) / item
                output_path.write_text(page)

        self.copy_assets()
        self.build_index()
        return self.duration

    @property
    def duration_str(self):
        """Return the duration as a human-readable string."""
        hours = self.duration // 60
        minutes = self.duration % 60
        return f"{hours}h {minutes}m"

    def to_dict(self):
        """Return a dictionary representation for courses.yml output."""
        link_path = self.output_folder.replace("web", "") + "/" + self.link

        data = {
            "description": self.description,
            "name": self.name,
            "cover": self.cover,
            "link": link_path,
            "duration": self.duration_str,
            "author": self.author,
            "groups": self.groups,
            "date_published": self.date_published,
            "level": self.level,
            "lessons": self.no_of_lessons,
        }
        # Only courses with a video get the key, so cards can test for it.
        if self.video:
            data["video"] = self.video
        if getattr(self, "projects", None):
            data["projects"] = self.projects
        return data
