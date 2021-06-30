import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

fileName = 'results/437v9.csv'
df = pd.read_csv(fileName, delimiter = ';') 


sns.set_theme(style="darkgrid") #darkgrid

#rs = np.random.RandomState(365)
#values = rs.randn(365, 4).cumsum(axis=0)
#dates = pd.date_range("1 1 2016", periods=365, freq="D")
#data = pd.DataFrame(values, dates, columns=["A", "B", "C", "D"])
#data = data.rolling(7).mean()
df2 = df.T

plt.figure(figsize=(10,6))
sns.lineplot(data=df2, palette="tab10", linewidth=2.0)
plt.ylabel("TotalTime", size=16)
plt.xlabel("Generation", size=16)

plt.xticks(ticks=np.arange(0,300,20), fontsize=12)
#plt.yticks(ticks=np.arange(df2[""],100,5), fontsize=12)

plt.title(fileName, size=24)