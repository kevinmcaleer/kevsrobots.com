import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import yaml

# Mapping of three-letter months to a single character
month_to_letter = {
    'Jan': 'J',
    'Feb': 'F',
    'Mar': 'M',
    'Apr': 'A',
    'May': 'M',
    'Jun': 'J',
    'Jul': 'J',
    'Aug': 'A',
    'Sep': 'S',
    'Oct': 'O',
    'Nov': 'N',
    'Dec': 'D'
}

label_size = 30
data_label_size = 30

# Robots
with open("web/_data/robots.yml", 'r') as stream:
    try:
        robots_data = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

# Projects     
with open("web/_data/projects.yml", 'r') as stream:
    try:
        projects_data = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

# Youtube      
with open("web/_data/youtube.yml", 'r') as stream:
    try:
        videos_data = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

# Posts        
with open("web/_data/posts.yaml", 'r') as stream:
    try:
        posts_data = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)
    posts_data = posts_data['posts']

# Reviews
with open("web/_data/reviews.yml", 'r') as stream:
    try:
        reviews_data = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

# Courses    
with open("web/_data/courses.yml", 'r') as stream:
    try:
        courses_data = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)
     
def print_raw_stats():
    print(f"Robots in the yaml file: {len(robots_data)}")   
    print(f"Projects in the yaml file: {len(projects_data)}")
    print(f"Videos: {len(videos_data)}")
    print(f"Posts: {len(posts_data)}")
    print(f"Reviews: {len(reviews_data)}")
    print(f"Courses: {len(courses_data)}")

robots = pd.DataFrame(robots_data)
projects = pd.DataFrame(projects_data)
videos = pd.DataFrame(videos_data)
posts = pd.DataFrame(posts_data)
reviews = pd.DataFrame(reviews_data)
courses = pd.DataFrame(courses_data)

# Convert dates to datetime
robots['date'] = pd.to_datetime(robots['date'])
projects['date'] = pd.to_datetime(projects['date'])
videos['date'] = pd.to_datetime(videos['published'])
posts['date'] = pd.to_datetime(posts['date'])
reviews['date'] = pd.to_datetime(reviews['date'])
courses['date'] = pd.to_datetime(courses['date_published'])

def produce_courses(year):
    # Create a DataFrame for all months of the year
    all_months = pd.DataFrame({'Month': pd.date_range(start=f'{year}-01-01', end=f'{year}-12-31', freq='M').strftime('%b')})

    courses_filtered = courses[courses['date'].dt.year == year]
    monthly_courses = courses_filtered.groupby(courses_filtered['date'].dt.to_period('M'))['name'].agg(['count', lambda x: ', '.join(x)]).reset_index()

    # Convert the period index to datetime for easier handling in plotting
    monthly_courses['Month'] = monthly_courses['date'].dt.strftime('%b')

    # Merge with all_months to include empty months
    course_year = pd.merge(all_months, monthly_courses, on='Month', how='left')
    course_year['count'] = course_year['count'].fillna(0)  # Replace NaN with 0

    # Produce the chart
    fig, ax = plt.subplots(figsize=(12, 6))
    bars = ax.bar(course_year['Month'], course_year['count'])
  
    # Set y-axis to only use whole numbers
    ax.yaxis.set_major_locator(MaxNLocator(integer=True))

    # Simplify the chart - remove labels, titles, and borders
    ax.set_xlabel('')
    ax.set_ylabel('')
    ax.set_title('')
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_visible(False)
    # ax.spines['bottom'].set_visible(False)
    ax.get_yaxis().set_visible(False)
    # ax.get_xaxis().set_ticks([])

    # Convert 'Month' from full abbreviation to single letter
    short_month_labels = course_year['Month'].map(month_to_letter)

    # Ensure the x-ticks match the length of your DataFrame
    ax.set_xticks(range(len(course_year)))
    ax.set_xticklabels(short_month_labels)
    ax.tick_params(axis='x', labelsize=label_size)
    ax.bar_label(bars, padding=3, fontsize=data_label_size)  # `padding=3` adds a small gap between the bar and label


    plt.savefig(f'web/assets/img/course_{year}.png')
    # plt.show()

def produce_reviews(year):
    # Create a DataFrame for all months of the year
    all_months = pd.DataFrame({'Month': pd.date_range(start=f'{year}-01-01', end=f'{year}-12-31', freq='M').strftime('%b')})

    reviews_filtered = reviews[reviews['date'].dt.year == year]
    monthly_reviews = reviews_filtered.groupby(reviews_filtered['date'].dt.to_period('M'))['title'].agg(['count', lambda x: ', '.join(x)]).reset_index()

    # Convert the period index to datetime for easier handling in plotting
    monthly_reviews['Month'] = monthly_reviews['date'].dt.strftime('%b')

    # Merge with all_months to include empty months
    review_year = pd.merge(all_months, monthly_reviews, on='Month', how='left')
    review_year['count'] = review_year['count'].fillna(0)  # Replace NaN with 0

    # Produce the chart
    fig, ax = plt.subplots(figsize=(12, 6))
    bars = ax.bar(review_year['Month'], review_year['count'])

    # Set y-axis to only use whole numbers
    ax.yaxis.set_major_locator(MaxNLocator(integer=True))

    # Simplify the chart - remove labels, titles, and borders
    ax.set_xlabel('')
    ax.set_ylabel('')
    ax.set_title('')
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_visible(False)
    # ax.spines['bottom'].set_visible(False)
    ax.get_yaxis().set_visible(False)
    # ax.get_xaxis().set_ticks([])
    ax.tick_params(axis='x', labelsize=label_size)
    # Convert 'Month' from full abbreviation to single letter
    short_month_labels = review_year['Month'].map(month_to_letter)
    ax.set_xticklabels(short_month_labels)
    ax.bar_label(bars, padding=3, fontsize=data_label_size)  # `padding=3` adds a small gap between the bar and label

    plt.savefig(f'web/assets/img/reviews_{year}.png')
    # plt.show()

def produce_posts(year):
    # Create a DataFrame for all months of the year
    all_months = pd.DataFrame({'Month': pd.date_range(start=f'{year}-01-01', end=f'{year}-12-31', freq='M').strftime('%b')})

    posts_filtered = posts[posts['date'].dt.year == year]
    monthly_posts = posts_filtered.groupby(posts_filtered['date'].dt.to_period('M'))['title'].agg(['count', lambda x: ', '.join(x)]).reset_index()

    # Convert the period index to datetime for easier handling in plotting
    monthly_posts['Month'] = monthly_posts['date'].dt.strftime('%b')

    # Merge with all_months to include empty months
    posts_year = pd.merge(all_months, monthly_posts, on='Month', how='left')
    posts_year['count'] = posts_year['count'].fillna(0)  # Replace NaN with 0

    # Produce the chart
    fig, ax = plt.subplots(figsize=(12, 6))
    bars = ax.bar(posts_year['Month'], posts_year['count'])

    # Set y-axis to only use whole numbers
    ax.yaxis.set_major_locator(MaxNLocator(integer=True))

    # Simplify the chart - remove labels, titles, and borders
    ax.set_xlabel('')
    ax.set_ylabel('')
    ax.set_title('')
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_visible(False)
    # ax.spines['bottom'].set_visible(False)
    ax.get_yaxis().set_visible(False)
    # ax.get_xaxis().set_ticks([])
    ax.tick_params(axis='x', labelsize=label_size)
    short_month_labels = posts_year['Month'].map(month_to_letter)
    ax.set_xticklabels(short_month_labels)
    ax.bar_label(bars, padding=3, fontsize=data_label_size)  # `padding=3` adds a small gap between the bar and label

    plt.savefig(f'web/assets/img/posts_{year}.png')
    # plt.show()

def produce_projects(year):
    # Create a DataFrame for all months of the year
    all_months = pd.DataFrame({'Month': pd.date_range(start=f'{year}-01-01', end=f'{year}-12-31', freq='M').strftime('%b')})

    projects_filtered = projects[projects['date'].dt.year == year]
    monthly_projects = projects_filtered.groupby(projects_filtered['date'].dt.to_period('M'))['name'].agg(['count', lambda x: ', '.join(x)]).reset_index()

    # Convert the period index to datetime for easier handling in plotting
    monthly_projects['Month'] = monthly_projects['date'].dt.strftime('%b')

    # Merge with all_months to include empty months
    projects_year = pd.merge(all_months, monthly_projects, on='Month', how='left')
    projects_year['count'] = projects_year['count'].fillna(0)  # Replace NaN with 0

    # Produce the chart
    fig, ax = plt.subplots(figsize=(12, 6))
    bars = ax.bar(projects_year['Month'], projects_year['count'])

    # Set y-axis to only use whole numbers
    ax.yaxis.set_major_locator(MaxNLocator(integer=True))

    # Simplify the chart - remove labels, titles, and borders
    ax.set_xlabel('')
    ax.set_ylabel('')
    ax.set_title('')
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_visible(False)
    # ax.spines['bottom'].set_visible(False)
    ax.get_yaxis().set_visible(False)
    # ax.get_xaxis().set_ticks([])
    short_month_labels = projects_year['Month'].map(month_to_letter)
    ax.set_xticklabels(short_month_labels)

    ax.tick_params(axis='x', labelsize=label_size)
    ax.bar_label(bars, padding=3, fontsize=data_label_size)  # `padding=3` adds a small gap between the bar and label

    plt.savefig(f'web/assets/img/projects_{year}.png')
    # plt.show()

def produce_videos(year):
    
    # Create a DataFrame for all months of the year
    all_months = pd.DataFrame({'Month': pd.date_range(start=f'{year}-01-01', end=f'{year}-12-31', freq='M').strftime('%b')})

    videos_filtered = videos[videos['date'].dt.year == year]
    monthly_videos = videos_filtered.groupby(videos_filtered['date'].dt.to_period('M'))['title'].agg(['count', lambda x: ', '.join(x)]).reset_index()

    # Convert the period index to datetime for easier handling in plotting
    monthly_videos['Month'] = monthly_videos['date'].dt.strftime('%b')

    # Merge with all_months to include empty months
    videos_year = pd.merge(all_months, monthly_videos, on='Month', how='left')
    videos_year['count'] = videos_year['count'].fillna(0)  # Replace NaN with 0

    # Produce the chart
    fig, ax = plt.subplots(figsize=(12, 6))
    bars = ax.bar(videos_year['Month'], videos_year['count'])

    # Set y-axis to only use whole numbers
    ax.yaxis.set_major_locator(MaxNLocator(integer=True))

    # Simplify the chart - remove labels, titles, and borders
    ax.set_xlabel('')
    ax.set_ylabel('')
    ax.set_title('')
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_visible(False)
    # ax.spines['bottom'].set_visible(False)
    ax.get_yaxis().set_visible(False)
    # ax.get_xaxis().set_ticks([])
    ax.tick_params(axis='x', labelsize=label_size)
    short_month_labels = videos_year['Month'].map(month_to_letter)
    ax.set_xticklabels(short_month_labels)
    ax.bar_label(bars, padding=3, fontsize=data_label_size)  # `padding=3` adds a small gap between the bar and label

    plt.savefig(f'web/assets/img/videos_{year}.png')
    # plt.show()

def produce_charts(year):
    produce_robots(year)
    produce_projects(year)
    produce_videos(year)
    produce_posts(year)
    produce_reviews(year)
    produce_courses(year)
    print_raw_stats()

def produce_robots(year):
    # Create a DataFrame for all months of the year
    all_months = pd.DataFrame({'Month': pd.date_range(start=f'{year}-01-01', end=f'{year}-12-31', freq='M').strftime('%b')})

    robots_filtered = robots[robots['date'].dt.year == year]
    monthly_robots = robots_filtered.groupby(robots_filtered['date'].dt.to_period('M'))['name'].agg(['count', lambda x: ', '.join(x)]).reset_index()

    # Convert the period index to datetime for easier handling in plotting
    monthly_robots['Month'] = monthly_robots['date'].dt.strftime('%b')

    # Merge with all_months to include empty months
    robots_year = pd.merge(all_months, monthly_robots, on='Month', how='left')
    robots_year['count'] = robots_year['count'].fillna(0)  # Replace NaN with 0

    # Produce the chart
    fig, ax = plt.subplots(figsize=(12, 6))
    bars = ax.bar(robots_year['Month'], robots_year['count'])

    # Set y-axis to only use whole numbers
    ax.yaxis.set_major_locator(MaxNLocator(integer=True))

    # Simplify the chart - remove labels, titles, and borders
    ax.set_xlabel('')
    ax.set_ylabel('')
    ax.set_title('')
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_visible(False)
    # ax.spines['bottom'].set_visible(False)
    ax.get_yaxis().set_visible(False)
    # ax.get_xaxis().set_ticks([])
    ax.tick_params(axis='x', labelsize=label_size)
    short_month_labels = robots_year['Month'].map(month_to_letter)
    ax.set_xticklabels(short_month_labels)
    ax.bar_label(bars, padding=3, fontsize=data_label_size)  # `padding=3` adds a small gap between the bar and label

    plt.savefig(f'web/assets/img/robots_{year}.png')
    # plt.show()

def monthly_stats(year):

    robots_filtered = robots[robots['date'].dt.year == year]
    monthly_robots = robots_filtered.groupby(robots_filtered['date'].dt.to_period('M'))['name'].agg(['count',lambda x: ', '.join(x)])

    print(monthly_robots)

# monthly_stats(2023)

produce_charts(2020)  
produce_charts(2021)  
produce_charts(2022)  
produce_charts(2023)
produce_charts(2024)     
