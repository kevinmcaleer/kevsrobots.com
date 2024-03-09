from course_builder.course import Course
import os
import yaml

class Courses():
    course_list = []
    output_folder = "web/learn"
    duration = 0

    def __init__(self, data=None):
        if data: 
            self.course_list = data

    def read_courses(self, course_folder):
        """ Read all the courses in the course folder """
        
        for course in os.listdir(course_folder):
            new_course = Course()
            if os.path.isdir(os.path.join(course_folder, course)):
                print(f'Found course: {course}')
                new_course.read_course(os.path.join(course_folder, course))
                new_course.output_folder = os.path.join(self.output_folder,course)
                self.course_list.append(new_course)
        return self

    def output_yml(self, output_folder):
        """ Output the course data to a yml file """

        # check if folder exists
        if not os.path.exists(output_folder):
            os.makedirs(output_folder)

        with open(f'{output_folder}/courses.yml', 'w') as outfile:

            c = []
            for course in self.course_list:
                c.append(course.__str__())

            yaml.safe_dump(c, outfile, default_flow_style=False)
        
    def build(self):
        """ Build the courses """
        
        for course in self.course_list:
            self.duration = course.build()
        
        self.build_index()
        self.build_recent()
    
    def build_index(self):

        # remove the index file if it exists
        if os.path.exists(f'{self.output_folder}/index.md'):
            os.remove(f'{self.output_folder}/index.md')
            
        index = '---' + "\n"
        index += 'layout: content' + "\n"
        index += 'title: Learn' + "\n"
        index += 'description: Take a course and learn something new' + "\n"
        index += f'duration: {self.duration}' + "\n"
        index += '---' + "\n\n"
        index += '{% include nav_courses.html %}' + "\n"
        index += '{% include breadcrumbs.html %}' + "\n"
        index += '{% include all_courses.html %}' + "\n"
        
        with open(f'{self.output_folder}/index.md', 'w') as build_file:
            build_file.writelines(index)

    def build_recent(self):
        """ Build the recent Courses page """

        # remove the index file if it exists
        if os.path.exists(f'{self.output_folder}/recent.md'):
            os.remove(f'{self.output_folder}/recent.md')
            
        index = '---' + "\n"
        index += 'layout: content' + "\n"
        index += 'title: Recent Courses' + "\n"
        index += 'description: Recent Courses' + "\n"
        index += '---' + "\n\n"
        index += '{% include nav_courses.html %}' + "\n"
        index += '{% include breadcrumbs.html %}' + "\n"
        index += '{% include recent_courses.html %}' + "\n"

        with open(f'{self.output_folder}/recent.md', 'w') as build_file:
            build_file.writelines(index)