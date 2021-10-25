from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og
from math import sqrt
from sys import argv
import argparse
import myFuncMulti as mfm
import ompl_demo as odm
import matplotlib.pyplot as plt
from scipy.ndimage.filters import gaussian_filter as scipy_gaussian
import numpy as np
import math
from scipy.ndimage.filters import gaussian_filter as scipy_gaussian
import scipy.ndimage
from scipy.interpolate import interp1d

pathFound, path = odm.plan(mfm.runTime, mfm.plannerType, mfm.objectiveType, mfm.fname, mfm.bound, mfm.start_pt, mfm.goal_pt)

if pathFound:
	pathList = mfm.getPathPointAsList(path)
	pathList = np.array(pathList) 		# pathList is numpy array now
else:
	print "Path not found..."

fig, ax= plt.subplots()
ax.axis('equal')

long_path = np.array([[0,0]])
print long_path.shape
f = interp1d(pathList[:,0], pathList[:,1], kind='quadratic')
x = np.linspace(0, 10, num=250, endpoint=True)
y = f(x)
for i in range(x.shape[0]):
	long_path = np.append(long_path, [[x[i], y[i]]], axis=0)
long_path = np.delete(long_path, 0, axis=0)

#leftBot_angle = math.pi/2
#rightBot_angle = -math.pi/2
bot1_angle = math.pi/4
bot2_angle = -math.pi/4
bot3_angle = 3*math.pi/4
bot4_angle = -3*math.pi/4

#left_dist = mfm.left_bot_dist_generator(long_path, leftBot_angle)
#right_dist = mfm.right_bot_dist_generator(long_path, rightBot_angle)
bot1_dist = mfm.left_bot_dist_generator(long_path, bot1_angle)
bot2_dist = mfm.right_bot_dist_generator(long_path, bot2_angle)
bot3_dist = mfm.left_bot_dist_generator(long_path, bot3_angle)
bot4_dist = mfm.right_bot_dist_generator(long_path, bot4_angle)


#leftBot_path = mfm.bot_path_generator(long_path, left_dist, leftBot_angle)
#rightBot_path = mfm.bot_path_generator(long_path, right_dist, rightBot_angle)
bot1_path = mfm.bot_path_generator(long_path, bot1_dist, bot1_angle)
bot2_path = mfm.bot_path_generator(long_path, bot2_dist, bot2_angle)
bot3_path = mfm.bot_path_generator(long_path, bot3_dist, bot3_angle)
bot4_path = mfm.bot_path_generator(long_path, bot4_dist, bot4_angle)

pathX, pathY = long_path.T
for i in range(len(bot1_path)):
	plt.cla()
	for obstacles in mfm.circularObstacles:
		ax.add_patch(plt.Circle((obstacles[0], obstacles[1]), obstacles[2], color='r'))
	ax.plot(pathX, pathY, 'b')
#	ax.add_patch(plt.Circle((leftBot_path[i][0], leftBot_path[i][1]), mfm.bot_rad, color='b'))
#	ax.add_patch(plt.Circle((rightBot_path[i][0], rightBot_path[i][1]), mfm.bot_rad, color='g'))
	ax.add_patch(plt.Circle((bot1_path[i][0], bot1_path[i][1]), mfm.bot_rad, color='b'))
	ax.add_patch(plt.Circle((bot2_path[i][0], bot2_path[i][1]), mfm.bot_rad, color='g'))
	ax.add_patch(plt.Circle((bot3_path[i][0], bot3_path[i][1]), mfm.bot_rad, color='b'))
	ax.add_patch(plt.Circle((bot4_path[i][0], bot4_path[i][1]), mfm.bot_rad, color='g'))

	vertices = np.array([[0, 0]])
	vertices = np.append(vertices, [[bot1_path[i][0], bot1_path[i][1]]], axis = 0)
	vertices = np.append(vertices, [[bot2_path[i][0], bot2_path[i][1]]], axis = 0)
	vertices = np.append(vertices, [[bot4_path[i][0], bot4_path[i][1]]], axis = 0)
	vertices = np.append(vertices, [[bot3_path[i][0], bot3_path[i][1]]], axis = 0)
	vertices = np.delete(vertices, [0], axis = 0)
	ax.add_patch(plt.Polygon(vertices, closed = True, fill = False))

	ax.plot()
	plt.pause(0.001)
	


plt.show()
exit()



