#!/usr/bin/env python3
# Course Builder
# Kevin McAleer
# October 2022

from course_builder.courses import Courses

courses = Courses()
courses.read_courses('source')
courses.build()
courses.output_yml('web/_data')
