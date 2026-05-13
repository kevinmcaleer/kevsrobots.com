import os
import shutil

import yaml

from course_builder.course import Course


class Courses:

    def __init__(self, data=None):
        self.course_list = data or []
        self.output_folder = "web/learn"
        self.duration = 0

    def read_courses(self, course_folder):
        """Read all the courses in the course folder."""
        for entry in sorted(os.listdir(course_folder)):
            entry_path = os.path.join(course_folder, entry)
            if not os.path.isdir(entry_path):
                continue

            course_yml = os.path.join(entry_path, "course.yml")
            if not os.path.exists(course_yml):
                continue

            print(f"Found course: {entry}")
            new_course = Course()
            result = new_course.read_course(entry_path)
            if result is None:
                print(f"Warning: skipping course {entry} (failed to load)")
                continue

            new_course.output_folder = os.path.join(self.output_folder, entry)
            self.course_list.append(new_course)
        return self

    def output_yml(self, output_folder):
        """Output the course data to a YAML file."""
        os.makedirs(output_folder, exist_ok=True)

        courses_data = [course.to_dict() for course in self.course_list]

        with open(os.path.join(output_folder, "courses.yml"), "w") as outfile:
            yaml.safe_dump(courses_data, outfile, default_flow_style=False)

    def build(self):
        """Build all courses."""
        self.duration = 0
        for course in self.course_list:
            result = course.build()
            if result is not None:
                self.duration += result

        self.build_index()
        self.build_recent()

    def build_index(self):
        """Copy the learn index page."""
        source = "web/_learn/index.md"
        destination = os.path.join(self.output_folder, "index.md")
        if os.path.exists(source):
            shutil.copy(source, destination)

    def build_recent(self):
        """Build the recent courses page."""
        recent_path = os.path.join(self.output_folder, "recent.md")

        content = (
            "---\n"
            "layout: content\n"
            "title: Recent Courses\n"
            "description: Learn something new. Today.\n"
            "---\n\n"
            "<div class=\"px-0 py-0 mx-0 my-0\">\n"
            "{% include breadcrumbs.html %}\n\n"
            "<div class=\"row\">\n"
            "<div class=\"col-12 col-md-3 col-lg-2\">\n"
            "{% include learn_sidebar.html %}\n"
            "</div>\n"
            "<div class=\"col-12 col-md-9 col-lg-10\">\n"
            "{% include recent_courses.html %}\n"
            "{{ content }}\n"
            "</div>\n</div>\n"
            
        )

        with open(recent_path, "w") as f:
            f.write(content)
