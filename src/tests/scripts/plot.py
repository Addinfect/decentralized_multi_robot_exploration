#!/usr/bin/env python3


from logging.handlers import DatagramHandler
from operator import concat
import matplotlib.pyplot as plt
import csv, os
import seaborn as sns
import numpy as np
import pandas as pd
df = pd.read_csv(os.path.join(os.path.expanduser('~'),'.ros/1_results.csv'), delimiter=" ")

"""world = "belgioioso"
assigner = ["Stupid", "Hungarian", "Auction"]
n_robots = range(2,7)
big_office = pd.DataFrame()
for assing_algo in assigner:
       for n in n_robots:
              data = df.loc[(df['World']==world) & (df['Assigner']==assing_algo) & (df['Number_Robots']==n)]
              data = data.rename(columns={'Time':'%s%d_time'%(assing_algo,n)})
              data = data.reset_index()
              #print(data.head())
              big_office = pd.concat([big_office, data['%s%d_time'%(assing_algo,n)]], axis=1)
#print(big_office.head())"""


sns.boxplot(data=df.loc[(df['World']=='belgioioso') & (df['Percentage']==95)], x="Number_Robots",y="Time", hue="Assigner")
fig = plt.figure()
sns.boxplot(data=df.loc[(df['World']=='belgioioso') & (df['Percentage']==95)], x="Number_Robots",y="Total_Distance", hue="Assigner")
fig = plt.figure()
sns.boxplot(data=df.loc[(df['World']=='belgioioso') & (df['Assigner']=='Hungarian')],y="Percentage", x="Time", hue="Number_Robots", orient="h")
fig = plt.figure()
sns.boxplot(data=df.loc[(df['World']=='belgioioso') & (df['Assigner']=='Hungarian')],y="Percentage", x="Time", hue="Number_Robots", orient="h")


plt.show()
