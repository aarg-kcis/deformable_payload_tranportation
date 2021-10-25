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

fig, ax = plt.subplots()
ax.axis('equal')

xs = np.linspace(0, 10, num=500, endpoint=True)
long_path = np.empty([xs.shape[0], 2])
for i in range(xs.shape[0]):
	long_path[i] = [xs[i], 0]

leftBot_angle = math.pi/2
rightBot_angle = -math.pi/2

left_dist, left_disto = mfm.left_bot_dist_generator(long_path, leftBot_angle)
right_dist = mfm.right_bot_dist_generator(long_path, rightBot_angle)

#leftDist = np.minimum(left_dist, np.minimum(bot1_dist, bot3_dist))
#rightDist = np.minimum(right_dist, np.minimum(bot2_dist, bot4_dist))


leftBot_patho = mfm.bot_path_generator(long_path, left_disto, leftBot_angle)
leftBot_path = mfm.bot_path_generator(long_path, left_dist, leftBot_angle)
rightBot_path = mfm.bot_path_generator(long_path, right_dist, rightBot_angle)


ax.add_patch(plt.Rectangle((4, 0.5), 2,1, color='b'))
ax.plot([0],[-0.5])

long_path = long_path[0::5]
pathX, pathY = long_path.T
ax.plot(pathX, pathY, '.g')

leftBot_path  = leftBot_path[0::5]
bot1x, bot1y = leftBot_path.T
ax.plot(bot1x, bot1y, '.-b')

leftBot_patho  = leftBot_patho[0::5]
bot1x, bot1y = leftBot_patho.T
ax.plot(bot1x, bot1y, '.r')

rightBot_path  = rightBot_path[0::5]
bot1x, bot1y = rightBot_path.T
ax.plot(bot1x, bot1y, '.-y')

ax.add_patch(plt.Circle((leftBot_path[10][0], leftBot_path[10][1]), mfm.bot_rad, color='b'))
ax.add_patch(plt.Circle((rightBot_path[10][0], rightBot_path[10][1]), mfm.bot_rad, color='r'))
#o = np.loadtxt('leftBot_dist_o.txt')
#p = np.loadtxt('leftBot_dist.txt')
#plt.plot(o, '.')
#plt.plot(p)
ax.plot()
plt.show()

for i in range(len(rightBot_path)):
	raw_input()
	plt.cla()
	for obstacles in mfm.leftRectangularObstacles:
		ax.plot([obstacles[0], obstacles[2]], [obstacles[1], obstacles[3]])
#	for obstacles in mfm.rightRectangularObstacles:
#		ax.plot([obstacles[0], obstacles[2]], [obstacles[1], obstacles[3]])

	ax.add_patch(plt.Rectangle((4, 0.5), 2,1.8, color='b'))
#	ax.add_patch(plt.Rectangle((2, 0.75), 2,-1.0, color='b'))
#	ax.add_patch(plt.Rectangle((8, 0.4), 2,2.1, color='b'))
#	ax.add_patch(plt.Rectangle((2, -2.5), 4,1.8, color='b'))
#	ax.add_patch(plt.Rectangle((6, -2.5), 2,1.0, color='b'))
#	ax.add_patch(plt.Rectangle((8, -2.5), 2,2.1, color='b'))
	ax.add_patch(plt.Circle((0,0), mfm.bot_rad/2, color='r'))
	ax.add_patch(plt.Circle((10,0), mfm.bot_rad/2, color='r'))
	
	ax.plot(pathX, pathY, 'b')
	ax.add_patch(plt.Circle((pathX[i], pathY[i]), mfm.bot_rad/2, color='r'))
	
	ax.add_patch(plt.Circle((leftBot_path[i][0], leftBot_path[i][1]), mfm.bot_rad, color='b'))
	ax.add_patch(plt.Circle((rightBot_path[i][0], rightBot_path[i][1]), mfm.bot_rad, color='g'))

	ax.plot([long_path[i][0], rightBot_path[i][0]], [long_path[i][1], rightBot_path[i][1]])
	ax.plot([long_path[i][0], leftBot_path[i][0]], [long_path[i][1], leftBot_path[i][1]])

	ax.plot()
	plt.pause(0.001)
	



plt.show()
exit()



