#!/usr/bin/env python
# coding: utf-8


from matplotlib import pyplot as plt
import matplotlib
import sys
from math import sin, cos, pi
import numpy as np
import datetime
import pandas as pd

matplotlib.use('TkAgg')

df = pd.read_csv('results/costs.csv', delimiter=' ')

df['Time'] = pd.to_datetime(df['Time'],errors='coerce', format='%H:%M').dt.time
df = df.sort_values(by='Time')
print (df['Time'])

print(df.columns)
headers = df.columns[1:]

fig, (ax0, ax1, ax2) = plt.subplots(nrows=3)

dict_axes = {'bed': ax0, 'tableD': ax1, 'board': ax2}
dict_colors = {'bed': 'red', 'tableD': 'green', 'board': 'blue'}

yticks = np.arange(start=0, stop=3.5, step=0.5)

# index = pd.Index(df['Time'][::12])
index = pd.Index(df['Time'][::4])

all_labels = df['Time'].apply(lambda x: x.strftime('%H:%M'))

# labels = all_labels[::12]
labels = all_labels[::4]

for ax in [ax0, ax1, ax2]:
    ax.yaxis.set_ticks(yticks)
    ax.tick_params(axis='both', which='major', labelsize=12)
    ax.spines['right'].set_visible(False)
    ax.spines['top'].set_visible(False)
    ax.set_xticks(index)
    ax.set_xticklabels(labels)

for h in headers:
    dict_axes[h].plot('Time', h, data=df, color=dict_colors[h], label= h)

fig.subplots_adjust(hspace=1.0)
fig.legend()
plt.show()
