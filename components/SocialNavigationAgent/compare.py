import sys
from math import sin, cos, pi
import numpy as np
import matplotlib.pyplot as plt

with open(sys.argv[1], 'r') as f:
    ax = []
    ay = []
    lines = f.readlines()
    for l in lines:
        parts = l.split()
        x = parts[0]
        y = parts [1]
        ax.append(x)
        ay.append(y)
plt.plot(ax, ay, 'r-')

with open(sys.argv[2], 'r') as f:
    ax = []
    ay = []
    lines = f.readlines()
    for l in lines:
        parts = l.split()
        x = parts[0]
        y = parts [1]
        ax.append(x)
        ay.append(y)
plt.plot(ax, ay, 'r-')

with open(sys.argv[3], 'r') as f:
    ax = []
    ay = []
    lines = f.readlines()
    for l in lines:
        parts = l.split()
        x = parts[0]
        y = parts [1]
        ax.append(x)
        ay.append(y)
plt.plot(ax, ay, 'r-')
#plt.plot(ax, ay, 'go')

with open(sys.argv[4], 'r') as f:
    ax = []
    ay = []
    ath = []
    lines = f.readlines()
    for l in lines:
        parts = l.split()
        x = float(parts[0])
        y = float(parts [1])
        th = float(parts [2])

        ax.append(x)
        ay.append(y)
        ath.append(th)

    for i in range(len(ax)):
        body = plt.Circle((ax[i], ay[i]), radius=0.35, fill=False)
        plt.gca().add_patch(body)


        x_aux = ax[i] + 0.35* cos(pi/2 - ath[i]);

        y_aux = ay[i] + 0.35 * sin(pi/2 - ath[i]);

        heading = plt.Line2D((ax[i], x_aux), (ay[i], y_aux), lw=1, color='k')
        plt.gca().add_line(heading)


with open(sys.argv[5], 'r') as f:
    ax = []
    ay = []
    lines = f.readlines()
    for l in lines:
        parts = l.split()
        x = parts[0]
        y = parts [1]
        ax.append(x)
        ay.append(y)
plt.plot(ax, ay, 'g.')


plt.xlim([0,10])
plt.ylim([0,10])
plt.axis('equal')
plt.show()
