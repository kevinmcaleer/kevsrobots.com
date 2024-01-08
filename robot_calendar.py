import yaml
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import matplotlib.colors as mcolors

def read_yaml_file(filename):
    with open(filename, 'r') as file:
        return yaml.safe_load(file)

month_names = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun', 'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec']

def week_number(date):
    date = date.date()
    return (date - date.replace(month=1, day=1)).days // 7 + 1

from math import ceil

import matplotlib.patches as patches

def create_calendar_heatmap(posts, year):
    # Create an empty DataFrame with dates of the specified year
    dates = pd.date_range(start=f'{year}-01-01', end=f'{year}-12-31', freq='D')
    calendar_df = pd.DataFrame({'Date': dates, 'Post': np.zeros(len(dates), dtype=int)})

    # Mark the dates with blog posts
    for post in posts:
        post_date_str = post['date']
        post_date = pd.to_datetime(post_date_str).date()
        print(f"Post date: {post_date}")

        if post_date.year == year:
            calendar_df.loc[calendar_df['Date'] == pd.Timestamp(post_date), 'Post'] = 1

    # Group the DataFrame by weeks
    calendar_df['Week'] = calendar_df['Date'].dt.isocalendar().week
    calendar_df['Day'] = calendar_df['Date'].dt.dayofweek

    # Pivot the DataFrame to have weeks as columns and days of the week as rows
    heatmap_df = calendar_df.pivot_table(values='Post', index='Day', columns='Week', fill_value=0, aggfunc='sum')

    # If the last week number is 1, it belongs to the next year. Rename it to the next integer after the previous week number.
    if heatmap_df.columns[-1] == 1:
        heatmap_df.rename(columns={1: heatmap_df.columns[-2] + 1}, inplace=True)

    # Create a custom colormap to mimic GitHub contribution colors
    # github_colors = ['#ebedf0', '#9be9a8', '#40c463', '#30a14e', '#216e39']
    github_colors = ['#ebedf0', '#9be9a8', '#40c463']
    cmap = mcolors.LinearSegmentedColormap.from_list("", github_colors)

    # Plot the heatmap
    plt.figure(figsize=(16, 2))
    ax = plt.gca()
    plt.pcolor(heatmap_df, cmap=cmap, edgecolors='white', linewidths=3, alpha=0.85)
    ax.invert_yaxis()

    # Set x-axis tick positions and labels to the first week of each month
    month_names = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun', 'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec']
    first_weeks = [calendar_df.loc[calendar_df['Date'].dt.month == m, 'Week'].min() for m in range(1, 13)]
    plt.xticks(np.array(first_weeks) - 0.5, month_names)

    plt.yticks(np.arange(0.5, len(heatmap_df.index), 1), ['Mon', '', 'Wed', '', 'Fri', '', 'Sun'])


    # Adjust the spacing between the heatmap and the axis labels
    ax.tick_params(axis='both', which='both', length=0, pad=8)

    plt.title(f'Robot Build Calendar ({year})')
    ax = plt.gca()
    ax.set_frame_on(False)
    print("Saving robot heatmap.png")
    plt.savefig('web/assets/img/robotheatmap.png', dpi=300, bbox_inches='tight')
    # plt.show()

def main():
    # Read YAML file
    filename = 'web/_data/robots.yml'
    data = read_yaml_file(filename)

    # Specify the year for which the heatmap should be generated
    year = 2024

    # Create calendar heatmap
    create_calendar_heatmap(data, year)

if __name__ == '__main__':
    main()
