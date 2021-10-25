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

def generate_bot_paths():
	pathFound, path = odm.plan(mfm.runTime, mfm.plannerType, mfm.objectiveType, mfm.fname, mfm.bound, mfm.start_pt, mfm.goal_pt)
	
	if pathFound:
		pathList = mfm.getPathPointAsList(path)
		pathList = np.array(pathList) 		# pathList is numpy array now
	else:
		print "Path not found..."
	
	long_path = np.array([[0,0]])
	print long_path.shape
	f = interp1d(pathList[:,0], pathList[:,1], kind='quadratic')
	x = np.linspace(0, 10, num=500, endpoint=True)
	y = f(x)
	for i in range(x.shape[0]):
		long_path = np.append(long_path, [[x[i], y[i]]], axis=0)
	long_path = np.delete(long_path, 0, axis=0)
	
#	np.savetxt('./allPaths/long_path.txt', long_path)
	print "Loaded the previous path... "
	long_path = np.loadtxt('./allPaths/long_path.txt')
	
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

	np.savetxt('./allPaths/bot1_path.txt', bot1_path)
	np.savetxt('./allPaths/bot2_path.txt', bot2_path)
	np.savetxt('./allPaths/bot3_path.txt', bot3_path)
	np.savetxt('./allPaths/bot4_path.txt', bot4_path)
	np.savetxt('./allPaths/botl_path.txt', leftBot_path)
	np.savetxt('./allPaths/botr_path.txt', rightBot_path)
	
	return long_path, leftBot_path, rightBot_path, bot1_path, bot2_path, bot3_path, bot4_path



def main():
	fig, ax= plt.subplots()
	ax.axis('equal')
	
	long_path, leftBot_path, rightBot_path, bot1_path, bot2_path, bot3_path, bot4_path = generate_bot_paths()
	pathX, pathY = long_path.T
	#for i in range(len(rightBot_path)):
	#	plt.cla()
	#	for obstacles in mfm.circularObstacles:
	#		ax.add_patch(plt.Circle((obstacles[0], obstacles[1]), obstacles[2], color='r'))
	#	ax.plot(pathX, pathY, 'b')
	#	ax.add_patch(plt.Circle((pathX[i], pathY[i]), mfm.bot_rad/2, color='r'))
	#	
	#	ax.add_patch(plt.Circle((leftBot_path[i][0], leftBot_path[i][1]), mfm.bot_rad, color='b'))
	#	ax.add_patch(plt.Circle((rightBot_path[i][0], rightBot_path[i][1]), mfm.bot_rad, color='g'))
	#	ax.add_patch(plt.Circle((bot1_path[i][0], bot1_path[i][1]), mfm.bot_rad, color='b'))
	#	ax.add_patch(plt.Circle((bot2_path[i][0], bot2_path[i][1]), mfm.bot_rad, color='g'))
	#	ax.add_patch(plt.Circle((bot3_path[i][0], bot3_path[i][1]), mfm.bot_rad, color='b'))
	#	ax.add_patch(plt.Circle((bot4_path[i][0], bot4_path[i][1]), mfm.bot_rad, color='g'))
	#
	#	ax.plot([long_path[i][0], rightBot_path[i][0]], [long_path[i][1], rightBot_path[i][1]])
	#	ax.plot([long_path[i][0], leftBot_path[i][0]], [long_path[i][1], leftBot_path[i][1]])
	#	ax.plot([long_path[i][0], bot1_path[i][0]], [long_path[i][1], bot1_path[i][1]])
	#	ax.plot([long_path[i][0], bot2_path[i][0]], [long_path[i][1], bot2_path[i][1]])
	#	ax.plot([long_path[i][0], bot3_path[i][0]], [long_path[i][1], bot3_path[i][1]])
	#	ax.plot([long_path[i][0], bot4_path[i][0]], [long_path[i][1], bot4_path[i][1]])
	#
	#	vertices = np.array([[0, 0]])
	#	vertices = np.append(vertices, [[leftBot_path[i][0], leftBot_path[i][1]]], axis = 0)
	#	vertices = np.append(vertices, [[bot1_path[i][0], bot1_path[i][1]]], axis = 0)
	#	vertices = np.append(vertices, [[bot2_path[i][0], bot2_path[i][1]]], axis = 0)
	#	vertices = np.append(vertices, [[rightBot_path[i][0],rightBot_path[i][1]]], axis = 0)
	#	vertices = np.append(vertices, [[bot4_path[i][0], bot4_path[i][1]]], axis = 0)
	#	vertices = np.append(vertices, [[bot3_path[i][0], bot3_path[i][1]]], axis = 0)
	#	vertices = np.delete(vertices, [0], axis = 0)
	#	ax.add_patch(plt.Polygon(vertices, closed = True, fill = False))
	#
	#	ax.plot()
	#	plt.pause(0.001)
		
	
	
	bot1x, bot1y = bot1_path.T
	ax.plot(bot1x, bot1y, 'go-')
	bot2x, bot2y = bot2_path.T
	ax.plot(bot2x, bot2y, 'ro-')
	
	plt.show()

if __name__ == '__main__':
	main()
