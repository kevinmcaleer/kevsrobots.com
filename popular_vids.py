import pandas as pd
import numpy as np
import yaml

# Read in the data
videos = pd.read_csv('Table data.csv')

videos = videos.drop([0])
videos = videos.drop(['Video title','Impressions',
                      'Impressions click-through rate (%)',
                      'Average view duration',
                      'Estimated revenue (GBP)',
                      'Video publish time',
                      'Subscribers',
                      'Watch time (hours)',
                      ], axis=1)
videos = videos[videos['Views'].notna()]
videos['Views'] = videos['Views'].astype(int)


# Calculate the 90th percentile of views
percentile_90 = videos['Views'].quantile(0.9)

# Add a new column for the top 10% flag
videos['popular'] = videos['Views'].apply(lambda x: 'True' if x >= percentile_90 else 'False')

# Manually construct the YAML string
yaml_string = "\n".join(
    "- Content: {}\n  Views: {}\n  popular: {}".format(row['Content'], row['Views'], row['popular']) 
    for index, row in videos.iterrows()
)

# Write to a YAML file
with open('web/_data/popular_videos.yaml', 'w') as file:
    file.write(yaml_string)
