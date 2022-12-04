# Course Builder
Builds a list of courses from the source folder - output is a `courses.yml` file

## Course Builder
Builds individual courses.

Takes an input of: `course.yml` which contains a list of sections with a name and content. Content contains a list of markdown files

## course.py
This class models an individual course, used to read in the `course.yml` and:
* [x] build the course output in the output folder
* [x] builds the additional front matter, including percentage complete (`number of lessons` / `current lesson no`)
* [x] creates folders if they dont exist
* [x] counts the number of lessons in each section to provide an accurate `no_of_lesson` property

# build.py
builds all the courses from the source folder and:
* [x] outputs the `courses.yml` into the `/web/_data` folder
* [x] outputs the built course into the `/learn` folder

# courses.yml
* [x] The data file output to the `_data` folder
* [x] has the following attributes for each course:

* [x] yaml navigation structure
```yaml
- name: <course_name>
  link: <url link to course>
  description: description of course
  tags: any tags
  cover: <url to cover image>
  level: beginner, intermediate, advanced
  navigation:
    section: 
      name: Overview
      content:
      - introduction
      - what you'll learn
  
```
