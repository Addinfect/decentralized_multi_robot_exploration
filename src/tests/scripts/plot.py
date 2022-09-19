#!/usr/bin/env python3


from logging.handlers import DatagramHandler
from operator import concat
import matplotlib.pyplot as plt
import csv, os
import seaborn as sns
import numpy as np
import pandas as pd
df = pd.read_csv(os.path.join(os.path.expanduser('~'),'.ros/1_results.csv'), delimiter=" ")

world = "belgioioso"
assigner = ["Stupid", "Hungarian", "Auction"]
n_robots = range(2,7)
big_office = pd.DataFrame()
number_of_experiments = pd.DataFrame(index=[50,75,90,95,98])
for assing_algo in assigner:
       for n in n_robots:
              count = df.loc[(df['World']=='belgioioso') & (df["Assigner"] == assing_algo) & (df["Number_Robots"] == n)]["Percentage"].value_counts(dropna=True,sort=False)
              count = count.rename('%s_%d'%(assing_algo,n))
              number_of_experiments= pd.concat([number_of_experiments,count], axis=1)

print(number_of_experiments)

              
"""
              data = df.loc[(df['World']==world) & (df['Assigner']==assing_algo) & (df['Number_Robots']==n)]
              data = data.rename(columns={'Time':'%s%d_time'%(assing_algo,n)})
              data = data.reset_index()
              #print(data.head())
              big_office = pd.concat([big_office, data['%s%d_time'%(assing_algo,n)]], axis=1)
#print(big_office.head())"""


sns.boxplot(data=df.loc[(df['World']=='belgioioso') & (df['Percentage']==95)], x="Number_Robots",y="Time", hue="Assigner").set(title="Time @95%")
fig = plt.figure()
sns.boxplot(data=df.loc[(df['World']=='belgioioso') & (df['Percentage']==95)], x="Number_Robots",y="Total_Distance", hue="Assigner").set(title="Distance @95%")
fig = plt.figure()
sns.boxplot(data=df.loc[(df['World']=='belgioioso') & (df['Assigner']=='Hungarian')],y="Percentage", x="Time", hue="Number_Robots", orient="h").set(title="Hungarian Assigner")
fig = plt.figure()
#anzahl an durchführungen
#sns.boxplot(data=df.loc[(df['World']=='belgioioso') & (df['Assigner']=='Hungarian')],y="Percentage", x="Time", hue="Number_Robots", orient="h")
print(number_of_experiments.iloc[4])
sns.barplot(data=number_of_experiments.iloc[4], orient="h")
fig = plt.figure()
sns.barplot(data=number_of_experiments).set(title="Number of Expiriments")

fig = plt.figure()
sns.boxplot(data=df.loc[(df['World']=='belgioioso') & (df['Percentage']==50)], x="Number_Robots",y="Time", hue="Assigner").set(title="Time @50%")
fig = plt.figure()
sns.boxplot(data=df.loc[(df['World']=='belgioioso') & (df['Percentage']==98)], x="Number_Robots",y="Time", hue="Assigner").set(title="Time @98%")

#fig = plt.figure()
sns.set_theme(style="whitegrid")
# Initialize a grid of plots with an Axes for each walk
grid = sns.FacetGrid(df, col="Assigner", row="Number_Robots", palette="tab20c",
                     height=1.5)
# Draw a horizontal line to show the starting point
#grid.refline(y=0, linestyle=":")
# Draw a line plot to show the trajectory of each random walk
grid.map(sns.boxplot, "Percentage", "Time")
# Adjust the tick positions and labels
#grid.set(xlim=(50, 100), ylim=(50, 350))

# Adjust the arrangement of the plots
grid.fig.tight_layout(w_pad=1)
plt.show()
