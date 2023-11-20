import seaborn as sns
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import calendar
from datetime import datetime
from matplotlib.colors import LinearSegmentedColormap

data = pd.read_csv("Table data.csv")
# Convert 'Video publish time' to datetime
data['Video publish time'] = pd.to_datetime(data['Video publish time'])

data['Week'] = data['Video publish time'].dt.isocalendar().week
data['Day of Week'] = data['Video publish time'].dt.day_name()

# Filter the current year
current_year = datetime.now().year
data = data[data['Video publish time'].dt.year == current_year]

# Group by week and day of the week, then count videos
grouped_data = data.groupby(['Week', 'Day of Week']).size().reset_index(name='Video Count')

# pivot the data to get a matrix format
heatmap_data = grouped_data.pivot (index="Day of Week", columns="Week", values="Video Count").fillna(0)

# Sort the days of the week
days_order = ['Monday','Tuesday', 'Wednesday', 'Thursday', 'Friday', 'Saturday','Sunday']
heatmap_data = heatmap_data.reindex(days_order)

# Create a complete index for all the weeks of the current year
all_weeks = pd.Series(range(1,53))

# Redindex heatmap_data to include all weeks, filling missing weeks with zeros
heatmap_data = heatmap_data.reindex(columns=all_weeks, fill_value=0)

# Define your specific RGB colors (normalized to 0-1 range)
colors = [(235/255, 237/255, 240/255),  # light grey color
          (105/255, 193/255, 110/255),  # green 2 color
          (82, 155, 185),               # green 3 color
          (56,108, 62),
          (56,108, 62),
          (56,108, 62)]                 # green 4 color

# Create a custom colormap
custom_cmap = LinearSegmentedColormap.from_list("custom_cmap", colors)

# Set the size of the heatmap
plt.figure(figsize=(12, 6))

# Create the heatmap without annotations
ax = sns.heatmap(heatmap_data, cmap=custom_cmap, linewidths=.5, linecolor='white', cbar=False, square=True)

# Annotate with month names
for month in range(1, 13):
    month_start = datetime(current_year, month, 1).isocalendar()[1]
    plt.text(month_start, -1, datetime(current_year, month, 1).strftime('%b'), ha='center')

ax.set_xticks([])
ax.set_xlabel('')
ax.set_ylabel('')

plt.savefig('web/assets/img/yt_heatmap.png', dpi=300, bbox_inches='tight')

# plt.show()