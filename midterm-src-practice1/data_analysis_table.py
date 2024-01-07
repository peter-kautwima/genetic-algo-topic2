import pandas as pd

# Assuming df is your DataFrame
df = pd.read_csv('summary.csv')

# Sort the DataFrame
df_sorted = df.sort_values('vertical_distance', ascending=False)

# Calculate the correlation
correlations = df_sorted.drop('file', axis=1).corr()

print(df_sorted)
print(correlations)



"""
Here are some insights you can draw from this data:

Fitness and Distance Traveled: There seems to be a negative correlation between fitness and distance traveled. This could mean that in this context, lower distance traveled corresponds to higher fitness. This might be the case if the goal of the creatures is to achieve some task with minimal movement.

Number of Links and Joins: The number of links and joins are constant for most creatures, but the summary has significantly fewer links and no joins. This could suggest that the genetic algorithm is converging towards a solution with a specific number of links and joins.

Correlation Matrix: The last part of the output is a correlation matrix, which shows the pairwise correlation of each pair of variables. The values range from -1 to 1, where -1 indicates a perfect negative correlation, 1 indicates a perfect positive correlation, and 0 indicates no correlation. For example, the correlation between fitness and num_links is -0.993853, which is a strong negative correlation. This suggests that as the fitness increases, the number of links decreases.

Remember that these are just insights based on the provided data. The actual meaning and implications would depend on the specific context and details of the genetic algorithm and the simulation.
"""