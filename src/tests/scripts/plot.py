#!/usr/bin/env python3


from logging.handlers import DatagramHandler
import matplotlib.pyplot as plt
import seaborn, csv, os
import numpy as np
import pandas as pd
df = pd.read_csv(os.path.join(os.path.expanduser('~'),'.ros/000_results.csv'), delimiter=" ")

world = "office"
auction_2 = df.loc[(df['World']==world) & (df['Assigner']=='Auction') & (df['Number_Robots']==2)]
auction_3 = df.loc[(df['World']==world) & (df['Assigner']=='Auction') & (df['Number_Robots']==3)]
auction_4 = df.loc[(df['World']==world) & (df['Assigner']=='Auction') & (df['Number_Robots']==4)]
auction_5 = df.loc[(df['World']==world) & (df['Assigner']=='Auction') & (df['Number_Robots']==5)]
auction_6 = df.loc[(df['World']==world) & (df['Assigner']=='Auction') & (df['Number_Robots']==6)]

random_2 = df.loc[(df['World']==world) & (df['Assigner']=='Stupid') & (df['Number_Robots']==2)]
random_3 = df.loc[(df['World']==world) & (df['Assigner']=='Stupid') & (df['Number_Robots']==3)]
random_4 = df.loc[(df['World']==world) & (df['Assigner']=='Stupid') & (df['Number_Robots']==4)]
random_5 = df.loc[(df['World']==world) & (df['Assigner']=='Stupid') & (df['Number_Robots']==5)]
random_6 = df.loc[(df['World']==world) & (df['Assigner']=='Stupid') & (df['Number_Robots']==6)]

hungarian_2 = df.loc[(df['World']==world) & (df['Assigner']=='Hungarian') & (df['Number_Robots']==2)]
hungarian_3 = df.loc[(df['World']==world) & (df['Assigner']=='Hungarian') & (df['Number_Robots']==3)]
hungarian_4 = df.loc[(df['World']==world) & (df['Assigner']=='Hungarian') & (df['Number_Robots']==4)]
hungarian_5 = df.loc[(df['World']==world) & (df['Assigner']=='Hungarian') & (df['Number_Robots']==5)]
hungarian_6 = df.loc[(df['World']==world) & (df['Assigner']=='Hungarian') & (df['Number_Robots']==6)]

print("Auction Samples: 2:%d, 3:%d, 4:%d, 5:%d, 6:%d"\
       %(len(auction_2),len(auction_3),len(auction_4), len(auction_5), len(auction_6)))
print("Random Samples: 2:%d, 3:%d, 4:%d, 5:%d, 6:%d"\
       %(len(random_2),len(random_3),len(random_4), len(random_5), len(random_6)))
print("Hungarian Samples: 2:%d, 3:%d, 4:%d, 5:%d, 6:%d"\
       %(len(hungarian_2),len(hungarian_3),len(hungarian_4), len(hungarian_5), len(hungarian_6)))
# plot
D = ([auction_2['Time'].tolist(), auction_3['Time'].tolist(), auction_4['Time'].tolist(), auction_5['Time'].tolist(), auction_6['Time'].tolist()])
E = ([auction_2['Time'].tolist(), random_2['Time'].tolist(), hungarian_2['Time'].tolist()])


fig, ax = plt.subplots()
VP = ax.boxplot(D, positions=[1,2,3,4,5], widths=0.5, patch_artist=True,
                showmeans=False, showfliers=True,
                medianprops={"color": "white", "linewidth": 0.5},
                boxprops={"facecolor": "C0", "edgecolor": "white",
                          "linewidth": 0.5},
                whiskerprops={"color": "C0", "linewidth": 1.5},
                capprops={"color": "C0", "linewidth": 1.5}
                )
fig, ax = plt.subplots()
VP = ax.boxplot(E, positions=[1,2,3], widths=0.5, patch_artist=True,
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
