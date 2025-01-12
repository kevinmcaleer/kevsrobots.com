import yaml
import matplotlib.pyplot as plt

yaml_file = 'web/_data/yir.yml'

# Function to read YAML file
def read_yaml(file_path):
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    return data

# Function to plot the graph
def plot_growth(data):
    years = []
    subscribers = []
    targets = []

    for record in data:
        years.append(record['year'])
        subscribers.append(record.get('subscribers'))  # Use .get to handle missing subscribers
        targets.append(record['target'])

    # Separate subscribers and targets into different lists for plotting
    subscribers_valid = [sub if sub is not None else None for sub in subscribers]

    plt.figure(figsize=(10, 6))

    # Plot subscribers, ignoring None values
    plt.plot(
        [y for y, s in zip(years, subscribers_valid) if s is not None],
        [s for s in subscribers_valid if s is not None],
        marker='o',
        label='Subscribers',
        linestyle='-',
        linewidth=2,
    )

    # Plot targets
    plt.plot(years, targets, marker='s', label='Target', linestyle='--', linewidth=2)

    plt.title('Subscriber Growth: Actual vs Target', fontsize=16)
    plt.xlabel('Year', fontsize=14)
    plt.ylabel('Count', fontsize=14)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend(fontsize=12)
    plt.tight_layout()
    # plt.show()
    plt.savefig('web/assets/img/growth.png')

# Load YAML data from the file
data = read_yaml(yaml_file)

# Plot the graph
plot_growth(data)
