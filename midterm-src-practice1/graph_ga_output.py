import matplotlib.pyplot as plt
import pandas as pd

# Load the data from the CSV file
df = pd.read_csv("ga_output.csv")

# Create a new figure
plt.figure()

# Plot the maximum fitness at each generation
plt.plot(df["iteration"], df["fittest"], label="Max Fitness")

# Plot the average fitness at each generation
plt.plot(df["iteration"], df["mean"], label="Average Fitness")

# Add a legend
plt.legend()

# Add labels for the x and y axes
plt.xlabel("Generation")
plt.ylabel("Fitness")

# Show the plot
plt.show()