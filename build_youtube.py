import seaborn as sns
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import calendar
from datetime import datetime
from matplotlib.colors import LinearSegmentedColormap

# Load data
data = pd.read_csv("Table data.csv")

# Convert 'Video publish time' to datetime
data['Video publish time'] = pd.to_datetime(data['Video publish time'])

# Handle NaT values by either filling them or dropping them
data = data.dropna(subset=['Video publish time'])

# Calculate week number, ensuring Sunday is treated as part of the current week
data['Week'] = data['Video publish time'].apply(
    lambda x: x.isocalendar()[1] if x.weekday() != 6 else (x.isocalendar()[1] + 1)
)

# Adjust for edge cases where the week number might exceed 52
data['Week'] = data['Week'].apply(lambda x: 1 if x > 52 else x)

# Extract day name
data['Day of Week'] = data['Video publish time'].dt.day_name()

# Filter the data for the current year
current_year = datetime.now().year
data = data[data['Video publish time'].dt.year == current_year]

# Group by week and day of the week, then count videos
grouped_data = data.groupby(['Week', 'Day of Week']).size().reset_index(name='Video Count')

# Pivot the data to get a matrix format
heatmap_data = grouped_data.pivot(index="Day of Week", columns="Week", values="Video Count").fillna(0)

# Sort the days of the week
days_order = ['Monday', 'Tuesday', 'Wednesday', 'Thursday', 'Friday', 'Saturday', 'Sunday']
heatmap_data = heatmap_data.reindex(days_order)

# Create a complete index for all the weeks of the current year
all_weeks = pd.Series(range(1, 53))

# Reindex heatmap_data to include all weeks, filling missing weeks with zeros
heatmap_data = heatmap_data.reindex(columns=all_weeks, fill_value=0)

# Normalize colors to 0-1 range for the colormap
colors = [
    (235/255, 237/255, 240/255),  # light grey (no contribution)
    (105/255, 193/255, 110/255),  # light green (low contribution)
    (56/255, 108/255, 62/255)     # dark green (high contribution)
]

# Create a custom colormap with a linear gradient between the colors
custom_cmap = LinearSegmentedColormap.from_list("custom_cmap", colors, N=256)

# Set the size of the heatmap
plt.figure(figsize=(12, 6))

# Create the heatmap with explicit vmin and vmax
ax = sns.heatmap(
    heatmap_data, 
    cmap=custom_cmap, 
    linewidths=.5, 
    linecolor='white', 
    cbar=False, 
    square=True,
    vmin=0,  # Ensure 0 maps to the light grey
    vmax=heatmap_data.max().max()  # Scale the max value properly
)

# Annotate with month names
for month in range(1, 13):
    month_start = datetime(current_year, month, 1).isocalendar()[1]
    plt.text(month_start, -1, datetime(current_year, month, 1).strftime('%b'), ha='center')

ax.set_xticks([])
ax.set_xlabel('')
ax.set_ylabel('')

# Save the figure
plt.savefig('web/assets/img/yt_heatmap.png', dpi=300, bbox_inches='tight')

# plt.show()
