# Course Builder - course class
# Kevin McAleer
# October 2022

import os
import yaml
import json
from shutil import copytree as copy, rmtree as rmdir, copyfile as copyfile
from course_builder.lesson import Lesson

class Course():
    name = ""
    description = ""
    author = ""
    date_created = ""
    date_published = ""
    content = []
    layout = ""
    type = "page"
    links = []
    output = ""
    output_folder = "web/build"
    course_folder = ""
    level = 'beginner'
    cover = "" # cover image
    link = "" # url to first file in course
    duration = 0 # duration of course in minutes

    def __init__(self, name=None, description=None, author=None, 
                 date_created=None, date_published=None, 
                 content=None, layout=None, type=None, 
                 links=None, output=None, output_folder=None, course_folder=None, duration=None):
        if name: self.name = name
        if description: self.description = description
        if author: self.author = author
        if date_created: self.date_created = date_created
        if date_published: self.date_published = date_published
        if content: self.content = content
        if layout: self.layout = layout
        if type: self.type = type
        if links: self.links = links
        if output: 
            self.output = output
        else:
            self.output = "nav.html"
        if output_folder: 
            self.output_folder = output_folder
        else:
            self.output_folder = "not_specified"
        if course_folder: self.course_folder = course_folder
        if duration: self.duration = duration

    def calculate_duration(self):
        """ Read in all the text, count the words and eastimate the reading time """
        duration = 0
        with open(f'{self.course_folder}/course.yml', 'r') as stream:
            try:
                course=yaml.safe_load(stream)
                print(f'Course manifest loaded')
            except yaml.YAMLError as exc:
                print(exc)
        
        course = course[0]
        for section in course['content']:
            name = section['section']
            print(f"Name is: {name['name']}")

            for item in section['section']['content']:
                print(item)
                duration += self.get_duration(item)

        print(f'Course duration is: {duration}')
        self.duration = duration
        return duration

    def get_duration(self,item):
        """ get the duration of the video """
        # get the duration from the lesson
        # return the duration in minutes
        
        with open(f'{self.course_folder}/{item}', 'r') as stream:
            try:
                file_content = stream.read()
                a = file_content.split("---")[1]
                lesson=yaml.safe_load(a)
            except yaml.YAMLError as exc:
                print(exc)
            print('lesson is: ', lesson['title'])
            try:
                duration = lesson['duration']
            except:
                duration = 0
            print(f'duration is: {duration}')
        return duration

    def read_course(self, course_folder):
        self.course_folder = course_folder
        with open(f'{course_folder}/course.yml', 'r') as stream:
            try:
                course=yaml.safe_load(stream)
                print(f'Course manifest loaded')
            except yaml.YAMLError as exc:
                print(exc)
        
        course = course[0]
        self.author = course['author']
        self.name = course['name']
        self.description = course['description']
        self.date_created = course['date_created']
        self.date_published = course['date_published']
        self.content = course['content']
        self.cover = course['cover']
        self.duration = self.calculate_duration()
        # print(course['content'])
        print(f'found {self.no_of_lessons} lessons')
        return self

    def build_index(self):
        # copy the first course file as index.md
        copyfile(self.output_folder + '/' + self.lessons[0], self.output_folder + '/index.md')

    def create_output(self, output_folder=None):
        """ Build the course """
        print(f'Building course: {self.name}')
        
        # Create the course folder
        if output_folder: self.output_folder = output_folder
        print("creating output folder: ", self.output_folder)
        if not os.path.exists(self.output_folder):
            os.makedirs(self.output_folder)

    def __str__(self):
        return f'Course name: {self.name}, with {self.no_of_lessons} lessons, layout is: {self.layout}'

    @property
    def lessons(self)->list:
        """ return a list of lessons """
        lesson_list = []
        for i in self.content:
            for item in i['section']['content']:
                lesson_list.append(item)
        return lesson_list
    
    def next_lesson(self, lesson_number):
        """ return the next lesson """
        print(f'lesson number is: {lesson_number}, {self.lessons[lesson_number-1]}')
        if lesson_number < self.no_of_lessons:
            next = self.lessons[lesson_number].replace('.md', '.html')
            print(f'next lesson is: {next}')
            return next
        else:
            return None

    def previous_lesson(self, lesson_number):
        """ return the next lesson """
        print(f'lesson is:{lesson_number}, {self.lessons[lesson_number-1]}')
        if lesson_number > 1:
            previous = self.lessons[lesson_number-2].replace('.md', '.html')
            print(f'previous lesson is: {previous}')
            return previous
        else:
            return None

    @property
    def no_of_lessons(self):
        # loop through the sections and count the number of lessons within each section
        count = 0
        for i in self.content:
            for _ in i['section']['content']:
                count += 1
        return count

    @staticmethod
    def find_nth(haystack:str, needle:str, n:int)->int:
        start = haystack.find(needle)
        while start >= 0 and n > 1:
            start = haystack.find(needle, start+len(needle))
            n -= 1
        return start

    def build_navigation(self):
        """ Build the navigation front matter """
        # output is a nice YML structure with friendly names for each section, with a link to that page

        # for this course, go through each section
            # for each section, go through each item
                # for each item open the file and read the name, append it to the dictionary

        with open(f'{self.course_folder}/course.yml', 'r') as stream:
            try:
                course=yaml.safe_load(stream)
                print(f'Course manifest loaded')
            except yaml.YAMLError as exc:
                print(exc)
        
        course = course[0]

        nav = []
        sections = []

        details = {'name':course['name']}
        nav.append(details)
        # print(f"course is {course['content']}")
        for section in course['content']:
            name = section['section']
            print(f"Name is: {name['name']}")
            
            item_record = []
            for item in section['section']['content']:
                print(item)

                # open the file and read the name
                
                with open(f'{self.course_folder}/{item}', 'r') as stream:
                    try:
                        file_content = stream.read()
                        a = file_content.split("---")[1]
                        lesson=yaml.safe_load(a)
                    except yaml.YAMLError as exc:
                        print(exc)
                    # print('lesson is: ', lesson['title'])
                item = item.replace('.md', '.html')
                # print(f'item is (should have .md replaced by .html): {item}')
                record = {'name': lesson['title'], 'link': item}
                # print(f'record is: {record}')
                item_record.append(record)
            sections.append({'section':name['name'], 'content':item_record})
        
        content = {'content':sections}
        nav.append(content)
        # print(f'nav {nav}')

        n = yaml.load(json.dumps(nav), Loader=yaml.FullLoader)
        yaml_navigation_structure = yaml.dump(n, sort_keys=False)

        return yaml_navigation_structure

    def update_front_matter(self, lesson_file, front_matter):
        
        # read the lesson file, find the front matter makers and remove them
        lesson_list = lesson_file.split('---', 2) # splits the string into 3 parts

        l = yaml.load(lesson_list[1], Loader=yaml.FullLoader)
        # print(f'l is {l}')

        # should now have atleast 3 sections
        # 1 is empty
        # 2 is the front matter
        # 3 is the content

        content = lesson_list[2]
        
        front_matter_list = front_matter.split('---', 2) # splits the string into 3 parts
        f = yaml.load(front_matter_list[1], Loader=yaml.FullLoader)
        # print(f'f is: {f}')

        # merge the existing front matter with the new front matter

         # get navigation front matter
        nav = self.build_navigation()

        merged_frontmatter = f
        merged_frontmatter.update(l)

        merged_frontmatter = yaml.dump(merged_frontmatter, sort_keys=False)

        page = "---\n" + merged_frontmatter + "navigation:\n" + nav +  "---\n" + content
        return page

    def copy_assets(self):
        """ copy the assets folder to the output folder """

        # check if the assets folder exists and remove it
        if os.path.exists(f'{self.output_folder}/assets'):
            rmdir(f'{self.output_folder}/assets')
            
        copy(self.course_folder + '/assets', self.output_folder + '/assets')

    def build_front_matter(self, item, lesson:Lesson())->int:
        """ build the front matter for the lesson and returns the duration """

        front_matter = f'---' + "\n"
        front_matter += f'layout: {self.layout}' + "\n"
        front_matter += f'title: {item}' + "\n"
        front_matter += f'author: {self.author}' + "\n"
        front_matter += f'type: {self.type}' + "\n"
        if lesson.previous_link is not None:
            front_matter += f'previous: {lesson.previous_link}' + "\n"
        if lesson.next_link is not None:
            front_matter += f'next: {lesson.next_link}' + "\n"
        front_matter += f'description: {self.description}' + "\n"
        front_matter += f'percent: {lesson.percentage}' + "\n"
        
        # update front matter
        lesson_file = f'{self.course_folder}/{item}'
        with open(lesson_file, 'r') as f:
            print(f'reading {lesson_file}')
            lines = f.read()
            # print(f'lines is: {lines}')
        
        # words per minute
        words_per_minute = 100

        # count the number of words in the lines
        lesson.word_count = len(lines.split())  
        lesson.duration = round(lesson.word_count // words_per_minute)

        if lesson.duration == 0:
            lesson.duration = 1

        front_matter += f'duration: {lesson.duration}' + "\n" 
        front_matter += f'---' + "\n"
        page = self.update_front_matter(lesson_file=lines, front_matter=front_matter)
        
        # write the file
        with open(f'{self.output_folder}/{item}', 'w') as build_file:
            build_file.writelines(page)

        return lesson.duration

    def build(self):
        """ Build the front matter for each content page, and output to the build folder """
        # create front matter
        # calculate percentage complete
        # get reading duration

        # TODO: refactor - make simpler
        
        # Create the front matter that is used to build the page navigation and previous, next buttons
        self.create_output(self.output_folder)
   
        # work out how much each page is as a percentage
        if self.no_of_lessons > 0:
            page_percent = 100 // self.no_of_lessons
        else:
            page_percent = 100

        if len(self.content) == 0:
            print("no content found")
            return

        # reset page_count
        page_count = 1

        for section in self.content:
            # Each section has a name and a content list of items
            just_item = section['section']    

            #  for each item in the content section 
            for item in just_item['content']:
                
                # create a lesson to hold the lesson details
                lesson = Lesson()

                if page_count == 1:
                    url = item.replace('.md', '.html')
                    self.link = url

                index = just_item['content'].index(item)
               
                print(f'item is: {item}, index is {index}, page_count is {page_count}')
           
                # item_percentage = page_percent * page_count
                lesson.percentage = page_percent * page_count

                if lesson.percentage > 100:
                    lesson.percentage = 100
                elif lesson.percentage < 100 and page_count == self.no_of_lessons:
                    lesson.percentage = 100

                # set the previous and next links
                lesson.next_link = self.next_lesson(page_count)
                lesson.previous_link = self.previous_lesson(page_count)
                
                # increment if it's a page
                if item in self.lessons and page_count < self.no_of_lessons: 
                    page_count += 1
                    print(f'page is: {item}, page_count is {page_count}')
                   
                duration = self.build_front_matter(item, lesson)
                self.duration += duration

        self.copy_assets() # copy the assets folder to the output folder
        self.build_index() # build the index page
        return self.duration

    @property
    def duration_str(self):
        """ return the duration as a string """
        
        hours = self.duration // 60
        minutes = self.duration % 60
        if hours > 0:
            duration = f'{hours}h {minutes}m'
        else:
            duration = f'{minutes}m'
        return duration 

    def __str__(self):
        """ provide a list of information for to build the courses.yml data file """

        cover_path = self.output_folder.replace('web','') + "/" + self.cover
        link_path = self.output_folder.replace('web','') + "/" + self.link
        
        return {'description':self.description, 'name':self.name, 'cover':cover_path, 'link':link_path, 'duration':self.duration_str, 'author':self.author}  
