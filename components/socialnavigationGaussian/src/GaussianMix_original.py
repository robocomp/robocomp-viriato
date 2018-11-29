# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from normal import Normal

def isEdge (z, x, y, h):
	if z[y, x] >= h:
		len_x = z.shape[1] - 1
		len_y = z.shape[0] - 1
		if x > 0:
			if z[y, x - 1] < h:
				return True
			if y > 0 and z[y - 1, x - 1] < h:
				return True
			if y < len_y and z[y + 1, x - 1] < h:
				return True
		if x < len_x:
			if z[y, x + 1] < h:
				return True
			if y > 0 and z[y - 1, x + 1] < h:
				return True
			if y < len_y and z[y + 1, x + 1] < h:
				return True
		if y > 0 and z[y - 1, x] < h:
			return True
		if y < len_y and z[y + 1, x] < h:
			return True
	return False

def filterEdges (z, h):
	result = np.zeros(z.shape, dtype = np.uint8)
	for y in range(z.shape[0]):
		for x in range(z.shape[1]):
			if isEdge(z, x, y, h):
				result[y, x] = 1
				print("result:")
				print(result, y, x)

	return result

h = 0.5
resolution = 0.1
limits = [[-10.0, 10.0], [-10.0, 10.0]]

normals = [Normal ( mu = [[1.28], [1.28]] , sigma = [360*3.1415/180, 2.0, 1.0, 1.3333333333333333] , elliptical = True ), Normal ( mu = [[1.88], [1.88]] , sigma = [290*3.1415/180, 2.0, 1.0, 1.3333333333333333] , elliptical = True ),  Normal ( mu = [[2.48], [1.67]] , sigma = [225*3.1415/180, 2.0, 1.0, 1.3333333333333333] , elliptical = True ), Normal ( mu = [[-1.22], [-2.12]] , sigma = [360*3.1415/180, 2.0, 1.0, 1.3333333333333333] , elliptical = True ), Normal ( mu = [[-2.12], [-2.12]] , sigma = [290*3.1415/180, 2.0, 1.0, 1.3333333333333333] , elliptical = True )]
_, z = Normal.makeGrid(normals, h, 2, limits = limits, resolution = resolution)
grid = filterEdges(z, h)


np.savetxt('log.txt', grid, fmt='%.4e')

plt.figure()
plt.imshow(z, shape = grid.shape, interpolation = 'none', aspect = 'equal', origin = 'lower', cmap = 'Greys', vmin = 0, vmax = 2)

plt.figure()
plt.imshow(grid, shape = grid.shape, interpolation = 'none', aspect = 'equal', origin = 'lower', cmap = 'Greys', vmin = 0, vmax = 2)

plt.show()