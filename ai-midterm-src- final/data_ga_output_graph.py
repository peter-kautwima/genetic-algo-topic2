import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_csv("ga_output.csv")

plt.figure()

plt.plot(df["iteration"], df["fittest"], label="Max Fitness")

plt.plot(df["iteration"], df["mean"], label="Average Fitness")

plt.legend()

plt.xlabel("Generation")
plt.ylabel("Fitness")

plt.show()