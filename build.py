#!/usr/bin/env python3
# Course Builder
# Kevin McAleer
# October 2022

from course_builder.courses import Courses

# copy the build files to the /learn folder for Jekyll to build

courses = Courses()
courses.read_courses('source')
courses.build()
courses.output_folder = 'web/learn'
print(courses.output_yml('web/_data'))
