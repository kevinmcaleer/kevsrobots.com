#!/usr/bin/env python3
# Course Builder
# Kevin McAleer
# October 2022

from course_builder.courses import Courses

courses = Courses()
courses.read_courses('source')
courses.build()
courses.output_folder = 'web/learn'
courses.output_yml('web/_data')

# Generate course-lessons manifest consumed by /course-stats dashboard.
# Kept as a separate module so it can also be run standalone.
import build_course_manifest
build_course_manifest.build()

# Social share cards (og:image) for every course, incl. the "I completed it"
# card used by /learn/<course>/completed.html. See course_builder/social_cards.py.
from course_builder.social_cards import generate as generate_social_cards
generate_social_cards()
