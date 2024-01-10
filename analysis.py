import pandas as pd
import numpy as np
import yaml
import matplotlib.pyplot as plt
from datetime import datetime

ROBOTS_DATA_FILE = 'web/_data/robots.yml'
PROJECTS_DATA_FILE = 'web/_data/projects.yml'
YOUTUBE_DATA_FILE = 'web/_data/youtube.yml'
YOUTUBE_ANALYTICS_FILE = 'Table Data.csv'
DAILY_SUBSCRIBERS_FILE = 'Chart data.csv'

START_YEAR = 2020

current_year = datetime.now().year

years = np.arange(START_YEAR, current_year + 1)

def read_yaml(file_path):
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    return data

# Read in the data
projects = read_yaml(PROJECTS_DATA_FILE)
robots = read_yaml(ROBOTS_DATA_FILE)
youtube_analytics = pd.read_csv(YOUTUBE_ANALYTICS_FILE)
youtube = read_yaml(YOUTUBE_DATA_FILE)
daily_subscribers = pd.read_csv(DAILY_SUBSCRIBERS_FILE)

# Convert the data to Dataframes
projects_data = pd.DataFrame(projects)
robots_data = pd.DataFrame(robots)
youtube_analytics_data = pd.DataFrame(youtube_analytics)
youtube_data = pd.DataFrame(youtube)
daily_subs_data = pd.DataFrame(daily_subscribers)

# Answer the questions
# - How Many Robots were created in each year?

def how_many_robots(values, year):
    """ Return the number of robots released in the given year """
    values['date'] = pd.to_datetime(values['date'])
    filtered_data = values[values['date'].dt.year == year]
    no_of_robots = len(filtered_data)
    return no_of_robots

def create_how_many_robots_chart():
    """ Create a bar chart showing the number of robots created per year"""
    how_many_robots_created_per_year = []
    for year in years:
        robots_created = how_many_robots(robots_data, year)
        how_many_robots_created_per_year.append(robots_created)

    fig, ax = plt.subplots()
    bars = ax.bar(years, how_many_robots_created_per_year, width=0.6, color='skyblue')

    # Add data labels above each bar
    for bar, value in zip(bars, how_many_robots_created_per_year):
        
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height(), str(value),
                ha='center', va='bottom', fontsize=10)
    ax.set_xticks(years)
    ax.set_title('Number of Robots Created Per Year')
    ax.set_yticks([])
    plt.box(False)
    plt.savefig('web/assets/img/blog/yir/robots_created_per_year.png', dpi=300)

def how_many_subscribers_per_month(values, year):
    """ Return the number of subscribers in the given year """
    
    values['Date'] = pd.to_datetime(values['Date'])
    filtered_data = values[values['Date'].dt.year == year]
    # print(filtered_data.head())

    # filtered_data = filtered_data[filtered_data['Subscription status'] == 'Subscribed']
    values.drop['Content','Video title','Video publish time']
    # return a datafram with the month and the number of subscribers by using teh max 'Subscriber Count' value and group by month
    subscribers_per_month = filtered_data.groupby(filtered_data['Date'].dt.month)['Views'].sum()

    print(subscribers_per_month)
    return subscribers_per_month

def year_on_year_growth(values):
    """ Return the year on year growth of subscribers """

    # values = values.drop(['Content','Video title','Video publish time'], axis=1)

    values['Date'] = pd.to_datetime(values['Date'])
    values['Year'] = values['Date'].dt.year

    print(f'values before filtering years: {values.count}')

    # values = values[values['Subscription status'] == 'Subscribed']
    values = values[values['Year'] >= START_YEAR]

    print(f'values after filtering years: {values.count}')

    print(values.head())
    # Group the data by year and calculate the sum of "Subscriber Count" for each year
    yearly_subscribers = values.groupby('Year')['Subscribers'].sum()

    print(f'Yearly Subscribers: {yearly_subscribers}')
    # Create a bar chart for year-on-year subscriber comparison
    fig, ax = plt.subplots()
    years = yearly_subscribers.index
    subscribers = yearly_subscribers.values
    ax.bar(years, subscribers, color='skyblue')

    # Add labels and title
    ax.set_xticks(years)
    ax.set_xlabel('Year')
    ax.set_ylabel('Total Subscribers')
    ax.set_title('Year-on-Year Subscriber Comparison')

    # Show the plot
    plt.savefig('web/assets/img/blog/yir/year_on_year_growth.png', dpi=300)
    plt.show()

def monthly_growth(values):
    values['Date'] = pd.to_datetime(values['Date'])
    values['Year'] = values['Date'].dt.year
    # Filter the data between the starting date and end date
    filtered_data = values[(values['Year'] >= START_YEAR)]

    # Extract the month and year from the "Date" column
    filtered_data['Month'] = filtered_data['Date'].dt.strftime('%Y-%m')

    # Group the filtered data by month and calculate the monthly growth in subscribers
    monthly_growth = filtered_data.groupby('Month')['Subscribers'].max() - filtered_data.groupby('Month')['Subscribers'].min()

    # Create a bar chart for monthly subscriber growth
    fig, ax = plt.subplots(figsize=(10, 6))
    months = monthly_growth.index
    growth = monthly_growth.values
    ax.bar(months, growth, color='skyblue')

    # Add labels and title
    ax.set_xticks(range(len(months)))
    ax.set_xticklabels(months, rotation=45)
    ax.set_xlabel('Month')
    ax.set_ylabel('Monthly Growth in Subscribers')
    ax.set_title(f'Monthly Subscriber Growth')

    # Show the plot
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    # create_how_many_robots_chart()
    # for year in years:
    #     how_many_subscribers_per_month(okrs_data, year)
    # year_on_year_growth(daily_subs_data)
    monthly_growth(daily_subs_data)