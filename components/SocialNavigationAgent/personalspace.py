#!/usr/bin/env python
# -*- coding: utf-8 -*-




import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm
from math import *
from mpl_toolkits.mplot3d import axes3d





class Person(object):
    x = 0
    y = 0
    th = 0

    xdot = 0
    ydot = 0

    _radius = 0.30

    """ Public Methods """

    def __init__(self, x=0, y=0, th=0):
        self.x = x
        self.y = y
        self.th = th

    def draw(self, drawPersonalSpace=False):

        if (drawPersonalSpace):

            # define grid.
            npts = 50
            x = np.linspace(self.x - 4, self.x + 4, npts)
            y = np.linspace(self.y - 4, self.y + 4, npts)

            X, Y = np.meshgrid(x, y)
            # plt.plot(X, Y, '*')

            Z = self._calculatePersonalSpace(X, Y)
            # print(Z)

            CS = plt.contour(X, Y, Z, 10)
            # surf = ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cm.coolwarm, linewidth=0, antialiased=False)

        # Corpo
        body = plt.Circle((self.x, self.y), radius=self._radius, fill=False)
        # plt.gca().add_patch(body)
        x_aux = self.x + self._radius * cos(self.th)
        y_aux = self.y + self._radius * sin(self.th)
        heading = plt.Line2D((self.x, x_aux), (self.y, y_aux), lw=3, color='k')
        # plt.gca().add_line(heading)

        # plt.axis('equal')
        # plt.show()

    """ Private Methods """

    def _calculatePersonalSpace(self, x, y):

        sigma_h = 2.0
        sigma_r = 1.0
        sigma_s = 4 / 3

        alpha = np.arctan2(y - self.y, x - self.x) - self.th - pi / 2
        nalpha = np.arctan2(np.sin(alpha), np.cos(alpha))  # Normalizando no intervalo [-pi, pi)

        sigma = np.copy(nalpha)
        for i in range(nalpha.shape[0]):
            for j in range(nalpha.shape[1]):
                sigma[i, j] = sigma_r if nalpha[i, j] <= 0 else sigma_h

        a = cos(self.th) ** 2 / 2 * sigma ** 2 + sin(self.th) ** 2 / 2 * sigma_s ** 2
        b = sin(2 * self.th) / 4 * sigma ** 2 - sin(2 * self.th) / 4 * sigma_s ** 2
        c = sin(self.th) ** 2 / 2 * sigma ** 2 + cos(self.th) ** 2 / 2 * sigma_s ** 2

        z = np.exp(-(a * (x - self.x) ** 2 + 2 * b * (x - self.x) * (y - self.y) + c * (y - self.y) ** 2))

        return z



#MODULO MAIN

plt.close('all')

#fig = plt.figure()
#ax = fig.add_subplot(111, projection='3d')

fig, ax = plt.subplots()
ax.grid(True)

#x = y = np.arange(-3.0, 3.0, 0.05)
#X, Y = np.meshgrid(x, y)
#zs = np.array([fun(x,y) for x,y in zip(np.ravel(X), np.ravel(Y))])
#Z = zs.reshape(X.shape)



p1 = Person(0, 0, 3*pi/4)
p1.draw(drawPersonalSpace=True)

#p2 = Person(0, 5)
#p2.draw(ax, drawPersonalSpace=True)

plt.xlabel('X')
plt.ylabel('Y')

#plt.xlim(-6, 6)
#plt.ylim(-6, 6)

plt.axis('equal')
plt.show()