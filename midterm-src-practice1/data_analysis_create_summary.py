import pandas as pd
import glob
import matplotlib.pyplot as plt

# Create an empty DataFrame to store all data
all_data = pd.DataFrame()

# Loop over all CSV files, sorted in ascending order by filename
for csv_file in sorted(glob.glob('*.csv')):
    if csv_file == 'ga_output.csv':
        continue  # Ignore ga_output.csv file

    # Read the CSV file, skipping any whitespace after the comma delimiter
    df = pd.read_csv(csv_file, header=None, skipinitialspace=True)
    df = df.iloc[:, :-1]  # Drop the last column

    # Convert all values to numeric, replacing any non-numeric values with NaN
    df = df.apply(pd.to_numeric, errors='coerce')

    # Calculate some statistics
    fitness = df.sum(axis=1).mean()  # Example: mean of sum of each row
    num_links = df.count(axis=1).mean()  # Example: mean number of non-NA values in each row
    vertical_distance = df[0].sum()  # Example: sum of values in first column

    # Add the statistics to the DataFrame
    all_data = pd.concat([all_data, pd.DataFrame([{
        'file': csv_file,
        'fitness': fitness,
        'num_links': num_links,
        'vertical_distance': vertical_distance
    }])], ignore_index=True)

# Save the DataFrame to a new CSV file
all_data.to_csv('summary.csv', index=False)

# Load the data from the CSV file
df = pd.read_csv("ga_output.csv")

# Create a new figure for the first graph
plt.figure()

# Plot the maximum fitness at each generation
plt.plot(df["iteration"], df["fittest"], label="Max Fitness")

# Plot the average fitness at each generation
plt.plot(df["iteration"], df["mean"], label="Average Fitness")

# Plot the vertical distance at each generation
plt.plot(all_data.index, all_data['vertical_distance'], label="Vertical Distance")

# Add a legend
plt.legend()

# Add labels for the x and y axes
plt.xlabel("Generation")
plt.ylabel("Fitness")

# Show the plot
plt.show()

# Create a new figure for the second graph
plt.figure(figsize=(10, 6))
plt.xlabel('Index')  # X-axis label
plt.ylabel('Value')  # Y-axis label
plt.title('Metrics over time for each CSV file')  # Graph title

# Plot the fitness, number of links, and vertical distance
plt.plot(all_data['fitness'], label='Fitness')
plt.plot(all_data['num_links'], label='Number of Links')
plt.plot(all_data['vertical_distance'], label='Vertical Distance')

# Add a legend
plt.legend()

# Show the plot
plt.show()