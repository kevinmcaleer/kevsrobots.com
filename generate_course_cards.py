#!/usr/bin/env python3
"""Generate the social share cards for every course.

See course_builder/social_cards.py. build.py runs this too, so you only need
it on its own after editing web/_data/courses.yml or a course cover by hand.

    python3 generate_course_cards.py
"""
from course_builder.social_cards import generate

if __name__ == "__main__":
    generate()
