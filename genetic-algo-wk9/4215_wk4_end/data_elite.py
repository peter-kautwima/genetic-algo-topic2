import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
df = pd.read_csv('summary.csv')

# Display basic statistics
print(df.describe())

# Plot fitness over time
plt.figure(figsize=(10, 6))
plt.plot(df['fitness'])
plt.title('Fitness over time')
plt.xlabel('Iteration')
plt.ylabel('Fitness')
plt.grid(True)
plt.show()

# Plot number of links over time
plt.figure(figsize=(10, 6))
plt.plot(df['num_links'])
plt.title('Number of links over time')
plt.xlabel('Iteration')
plt.ylabel('Number of links')
plt.grid(True)
plt.show()

# Plot distance traveled over time
plt.figure(figsize=(10, 6))
plt.plot(df['distance_traveled'])
plt.title('Distance traveled over time')
plt.xlabel('Iteration')
plt.ylabel('Distance traveled')
plt.grid(True)
plt.show()