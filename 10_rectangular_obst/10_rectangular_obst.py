from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og
from math import sqrt
from sys import argv
import argparse
import myFuncRect as mfm
import ompl_demo as odm
import matplotlib.pyplot as plt
from scipy.ndimage.filters import gaussian_filter as scipy_gaussian
import numpy as np
import math
from scipy.ndimage.filters import gaussian_filter as scipy_gaussian
import scipy.ndimage
from scipy.interpolate import interp1d

fig, ax = plt.subplots()
ax.axis('equal')

xs = np.linspace(0, 12, num=500, endpoint=True)
long_path = np.empty([xs.shape[0], 2])
for i in range(xs.shape[0]):
	long_path[i] = [xs[i], 0]

leftBot_angle = math.pi/2
rightBot_angle = -math.pi/2
bot1_angle = math.pi/6
bot2_angle = -math.pi/6
bot3_angle = 5*math.pi/6
bot4_angle = -5*math.pi/6

left_dist = mfm.left_bot_dist_generator(long_path, leftBot_angle)
right_dist = mfm.right_bot_dist_generator(long_path, rightBot_angle)
bot1_dist = mfm.left_bot_dist_generator(long_path, bot1_angle)
bot2_dist = mfm.right_bot_dist_generator(long_path, bot2_angle)
bot3_dist = mfm.left_bot_dist_generator(long_path, bot3_angle)
bot4_dist = mfm.right_bot_dist_generator(long_path, bot4_angle)

leftDist = np.minimum(left_dist, np.minimum(bot1_dist, bot3_dist))
rightDist = np.minimum(right_dist, np.minimum(bot2_dist, bot4_dist))


leftBot_path = mfm.bot_path_generator(long_path, leftDist, leftBot_angle)
rightBot_path = mfm.bot_path_generator(long_path, rightDist, rightBot_angle)
bot1_path = mfm.bot_path_generator(long_path, leftDist, bot1_angle)
bot2_path = mfm.bot_path_generator(long_path, rightDist, bot2_angle)
bot3_path = mfm.bot_path_generator(long_path, leftDist, bot3_angle)
bot4_path = mfm.bot_path_generator(long_path, rightDist, bot4_angle)

pathX, pathY = long_path.T
for i in range(len(rightBot_path)):
	print i
#	plt.cla()
	if i==2 or i==80 or i==130 or i ==185 or i == 245 or i == 312 or i==400 or i==495:
		for obstacles in mfm.leftRectangularObstacles:
			ax.plot([obstacles[0], obstacles[2]], [obstacles[1], obstacles[3]])
		for obstacles in mfm.rightRectangularObstacles:
			ax.plot([obstacles[0], obstacles[2]], [obstacles[1], obstacles[3]])
	
		ax.add_patch(plt.Rectangle((2, 0.7), 2,1.8, color='b'))
		ax.add_patch(plt.Rectangle((4, 1.5), 4,1.0, color='b'))
		ax.add_patch(plt.Rectangle((8, 0.4), 2,2.1, color='b'))
		ax.add_patch(plt.Rectangle((2, -2.5), 4,1.8, color='b'))
		ax.add_patch(plt.Rectangle((6, -2.5), 2,1.0, color='b'))
		ax.add_patch(plt.Rectangle((8, -2.5), 2,2.1, color='b'))
		ax.add_patch(plt.Circle((0,0), mfm.bot_rad/2, color='r'))
		ax.add_patch(plt.Circle((12,0), mfm.bot_rad/2, color='r'))
		
		ax.plot(pathX, pathY, 'b')
	#	ax.add_patch(plt.Circle((pathX[i], pathY[i]), mfm.bot_rad/2, color='r'))
		
		ax.add_patch(plt.Circle((leftBot_path[i][0], leftBot_path[i][1]), mfm.bot_rad, color='r'))
		ax.add_patch(plt.Circle((rightBot_path[i][0], rightBot_path[i][1]), mfm.bot_rad, color='r'))
		ax.add_patch(plt.Circle((bot1_path[i][0], bot1_path[i][1]), mfm.bot_rad, color='r'))
		ax.add_patch(plt.Circle((bot2_path[i][0], bot2_path[i][1]), mfm.bot_rad, color='r'))
		ax.add_patch(plt.Circle((bot3_path[i][0], bot3_path[i][1]), mfm.bot_rad, color='r'))
		ax.add_patch(plt.Circle((bot4_path[i][0], bot4_path[i][1]), mfm.bot_rad, color='r'))
	
		ax.plot([long_path[i][0], rightBot_path[i][0]], [long_path[i][1], rightBot_path[i][1]], 'g')
		ax.plot([long_path[i][0], leftBot_path[i][0]], [long_path[i][1], leftBot_path[i][1]], 'g')
		ax.plot([long_path[i][0], bot1_path[i][0]], [long_path[i][1], bot1_path[i][1]], 'g')
		ax.plot([long_path[i][0], bot2_path[i][0]], [long_path[i][1], bot2_path[i][1]], 'g')
		ax.plot([long_path[i][0], bot3_path[i][0]], [long_path[i][1], bot3_path[i][1]], 'g')
		ax.plot([long_path[i][0], bot4_path[i][0]], [long_path[i][1], bot4_path[i][1]], 'g')
	
		vertices = np.array([[0, 0]])
		vertices = np.append(vertices, [[leftBot_path[i][0], leftBot_path[i][1]]], axis = 0)
		vertices = np.append(vertices, [[bot1_path[i][0], bot1_path[i][1]]], axis = 0)
		vertices = np.append(vertices, [[bot2_path[i][0], bot2_path[i][1]]], axis = 0)
		vertices = np.append(vertices, [[rightBot_path[i][0],rightBot_path[i][1]]], axis = 0)
		vertices = np.append(vertices, [[bot4_path[i][0], bot4_path[i][1]]], axis = 0)
		vertices = np.append(vertices, [[bot3_path[i][0], bot3_path[i][1]]], axis = 0)
		vertices = np.delete(vertices, [0], axis = 0)
		ax.add_patch(plt.Polygon(vertices, closed = True, fill = False, color='g'))

		ax.plot()
		plt.pause(0.001)

plt.show()	


#bot1x, bot1y = bot3_path.T
#ax.plot(bot1x, bot1y, 'go-')
#bot2x, bot2y = bot4_path.T
#ax.plot(bot2x, bot2y, 'ro-')

plt.show()
exit()



