import pandas as pd
import glob
import matplotlib.pyplot as plt

# Create an empty DataFrame to store all data
all_data = pd.DataFrame()

# Loop over all CSV files
for csv_file in glob.glob('*.csv'):
    # Read the CSV file
    df = pd.read_csv(csv_file, header=None)

    # Calculate some statistics
    fitness = df.sum(axis=1).mean()  # Example: mean of sum of each row
    num_links = df.count(axis=1).mean()  # Example: mean number of non-NA values in each row
    num_joins = df[0].count()  # Example: number of non-NA values in first column
    distance_traveled = df[0].sum()  # Example: sum of values in first column

    # Add the statistics to the DataFrame
    all_data = pd.concat([all_data, pd.DataFrame([{
        'file': csv_file,
        'fitness': fitness,
        'num_links': num_links,
        'num_joins': num_joins,
        'distance_traveled': distance_traveled
    }])], ignore_index=True)

# Save the DataFrame to a new CSV file
all_data.to_csv('summary.csv', index=False)

# Plot the data
plt.figure(figsize=(10, 6))
plt.plot(all_data['fitness'], label='Fitness')
plt.plot(all_data['num_links'], label='Number of Links')
plt.plot(all_data['num_joins'], label='Number of Joins')
plt.plot(all_data['distance_traveled'], label='Distance Traveled')
plt.legend()
plt.show()