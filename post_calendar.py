"""_summary_
ChatGPT Prompt:
I want to create a python program that will take a yaml file as input that contains a 
list of  blog posts each with a name, date of publish and url, and then create a graphic 
that shows an annual calendar a green block if a post was posted on a day, otherwise its a grey block. the graphic will show weeks across the x axis and days of the week on the y axis
"""

# pip install pyyaml matplotlib pandas

""" Yaml file format:
posts:
  - name: "Post 1"
    date: "2023-01-01"
    url: "https://example.com/post-1"
  - name: "Post 2"
    date: "2023-01-15"
    url: "https://example.com/post-2"
  - name: "Post 3"
    date: "2023-02-08"
    url: "https://example.com/post-3"
"""


import yaml
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

def read_yaml_file(filename):
    with open(filename, 'r') as file:
        return yaml.safe_load(file)

def create_calendar_heatmap(posts, year):
    # Create an empty DataFrame with dates of the specified year
    dates = pd.date_range(start=f'{year}-01-01', end=f'{year}-12-31', freq='D')
    calendar_df = pd.DataFrame({'Date': dates, 'Post': np.zeros(len(dates), dtype=int)})

    # Mark the dates with blog posts
    for post in posts:
        # post_date = datetime.strptime(post['date'], '%Y-%m-%d').date()
        post_date = post['date']

        print(f"Post: {post['date']}, Date: {post_date.year}, title: {post['title']}")
        if post_date.year == year:
            calendar_df.loc[calendar_df['Date'] == post_date, 'Post'] = 1

    # Pivot the DataFrame to have weeks as columns and days of the week as rows
    calendar_df['Week'] = calendar_df['Date'].dt.isocalendar().week
    calendar_df['Day'] = calendar_df['Date'].dt.dayofweek
    heatmap_df = calendar_df.pivot_table(values='Post', index='Day', columns='Week', fill_value=0)

    # Plot the heatmap
    plt.figure(figsize=(20, 5))
    plt.pcolor(heatmap_df, cmap='Greens', edgecolors='grey', linewidths=2)
    plt.gca().invert_yaxis()
    plt.xticks(np.arange(0.5, len(heatmap_df.columns), 1), heatmap_df.columns)
    plt.yticks(np.arange(0.5, len(heatmap_df.index), 1), ['Monday', 'Tuesday', 'Wednesday', 'Thursday', 'Friday', 'Saturday', 'Sunday'])
    plt.xlabel('Weeks')
    plt.ylabel('Days of the Week')
    plt.title(f'Blog Posts Calendar ({year})')
    plt.colorbar(label='Number of Posts')
    plt.show()

def main():
    # Read YAML file
    filename = 'web/_data/posts.yaml'
    data = read_yaml_file(filename)

    # Specify the year for which the heatmap should be generated
    year = 2023

    # Create calendar heatmap
    create_calendar_heatmap(data['posts'], year)

if __name__ == '__main__':
    main()
