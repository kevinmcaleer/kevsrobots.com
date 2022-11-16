# Course Builder
Builds a list of courses from the source folder - output is a `courses.yml` file

## Course Builder
Builds individual courses.

Takes an input of: `course.yml` which contains a list of sections with a name and content. Content contains a list of markdown files

## course.py
This class models an individual course, used to read in the `course.yml` and:
* build the course output in the output folder
* builds the additional front matter, including percentage complete (`number of lessons` / `current lesson no`)
* creates folders if they dont exist
* counts the number of lessons in each section to provide an accurate `no_of_lesson` property


# build.py
builds all the courses from the source folder and:
* outputs the `courses.yml` into the `/web/_data` folder
* outputs the built course into the `/learn` folder
