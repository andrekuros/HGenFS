# -*- coding: utf-8 -*-
"""
Spyder Editor

Este é um arquivo de script temporário.
"""

import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

path =  "bin/problems/FSTSP_10_customer_problems/20140810T123";
problem = '443v9'

fileName = problem + '_main.csv'

df = pd.read_csv(fileName, delimiter = ';') 
truckCost = pd.read_csv(path + problem + "/tau.csv", delimiter = ',', header=None) 
droneCost = pd.read_csv(path + problem + "/tauPrime.csv", delimiter = ',', header=None) 

df = pd.read_csv(fileName, delimiter = ';') 




      
if True:
    #437V1 - 56.468
    #pathTruck = [0, 8, 4, 9,10,3,1,7,5,6,11  ]
    pathTruck = [0, 4, 2, 9, 10, 3, 1, 7, 5, 6, 11 ] 
    pathTruckTime = [0 for x in range(0,12)]
    pathDrone = [8]
    pathDepLand = [[0, 11]]

      
if True:
    #437V2 - 55.3926
    #pathTruck = [0, 8, 4, 9,10,3,1,7,5,6,11  ]
    pathTruck = [0, 9, 10, 6, 5, 1, 8, 7, 4, 11]
    pathTruckTime = [0 for x in range(0,12)]
    pathDrone = [3,8]
    pathDepLand = [[9, 8], [8, 11]]
    
      
if True:
    #437V2 - 55.3926
    #pathTruck = [0, 8, 4, 9,10,3,1,7,5,6,11  ]
    pathTruck = [0,  1, 6, 5,  7, 8, 9, 10, 11]
    pathTruckTime = [0 for x in range(0,12)]
    pathDrone = [2, 4, 3]
    pathDepLand = [[0, 7], [7, 9], [9,11]]
#[0, 8, 4, 2, 9, 10, 3, 1, 7, 5, 6, 11]

truckTime = 0
for i in range (1, len(pathTruck)):       
    truckTime = truckTime + truckCost.iloc[pathTruck[i-1]][pathTruck[i]]
    pathTruckTime[i] = truckTime
    #print("a: ", i-1, " b: ", i, " - ", truckCost.iloc[i-1][i])
    
droneTime = []
truckSyncTime = []
syncTime = []
for i in range (0, len(pathDrone)):       
    droneTime.append(droneCost.iloc[pathDepLand[i][0]][pathDrone[i]] + droneCost.iloc[pathDrone[i]][pathDepLand[i][1]]) 
    truckSyncTime.append(pathTruckTime[ pathTruck.index(pathDepLand[i][1]) ] - pathTruckTime[ pathTruck.index(pathDepLand[i][0]) ])
    syncTime.append(droneTime[-1] - truckSyncTime[-1])
    
syncTotal = [x if x > 0 else 0 for x in syncTime]
syncTotal = sum(syncTotal)
    
print("TruckTime",truckTime)
print("PAthTruck",pathTruckTime)

print("\nTruckLegsSync",truckSyncTime )
print("droneTime",droneTime)
print ("sync: ", syncTime)
print("Final: ", truckTime + syncTotal + len(pathDrone)*2 - (1 if pathDepLand[0][0] == 0 else 0))

#print(truckCost.iloc[0][2])
#for i in range()





# In[]:

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