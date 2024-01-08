import pandas as pd

df = pd.read_csv('summary.csv')

# Sort the DataFrame
df_sorted = df.sort_values('vertical_distance', ascending=False)

# Calculate the correlation
correlations = df_sorted.drop('file', axis=1).corr()

print(df_sorted)
print(correlations)
