#!/usr/bin/env python3


import matplotlib.pyplot as plt
import seaborn, csv, os
import numpy as np
import pandas as pd
df = pd.read_csv(os.path.join(os.path.expanduser('~'),'.ros/000_results.csv'), delimiter=" ")


print(df.head())
auction_2 = df.loc[(df['Assigner']=='Auction') & (df['Number_Robots']==2)]
auction_3 = df.loc[(df['Assigner']=='Auction') & (df['Number_Robots']==3)]
auction_4 = df.loc[(df['Assigner']=='Auction') & (df['Number_Robots']==4)]
auction_5 = df.loc[(df['Assigner']=='Auction') & (df['Number_Robots']==5)]
auction_6 = df.loc[(df['Assigner']=='Auction') & (df['Number_Robots']==6)]

D = ([auction_2['Time'].tolist(), auction_3['Time'].tolist(), auction_4['Time'].tolist(), auction_5['Time'].tolist(), auction_6['Time'].tolist()])
print(D)
# plot

fig, ax = plt.subplots()
VP = ax.boxplot(D, positions=[1,2,3,4,5], widths=0.5, patch_artist=True,
                showmeans=False, showfliers=True,
                medianprops={"color": "white", "linewidth": 0.5},
                boxprops={"facecolor": "C0", "edgecolor": "white",
                          "linewidth": 0.5},
                whiskerprops={"color": "C0", "linewidth": 1.5},
                capprops={"color": "C0", "linewidth": 1.5}
                )

ax.set(xlim=(0, 6), xticks=np.arange(0, 6),
       ylim=(100, 300), yticks=np.arange(100, 300,25))

plt.show()
