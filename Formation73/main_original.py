#!/usr/bin/python
from __future__ import division

import sys
import time
from math import radians, degrees, pi
from matplotlib import pyplot as plt
import matplotlib.colors as pltc

from formation import Pibot, Leader, Follower
from pose import Pose
import constants as C

def main():
	# start time to calculate dt
	time_start = time.time()
	counter = 0
	robots = []

	# plot variables
	plot_leader = []
	plot_followers = [[] for i in range(C.NO_OF_ROBOTS_IN_FORMATION)]

	# create instances for all the robots (ID = 0 as leader and others as followers initially)
	robots.append(Leader(0, Pose(0,0,radians(90))))
	for i in range(1, C.MAX_ROBOTS):
		robots.append(Follower(int(i), Pose(0,0,0), robots[0]))


	# Formation Graph (out of 20 robots)
	f_graph = {'l':1, 'f':range(1, 50)}

	dist = C.Lij
	print dist
	# run formation in the main loop
	while True:
		try:
			if counter > C.MAX_ITER:
				break
			counter  = counter + 1
			dist = dist - 0.001 #Subhasis to change
#			for r in range(C.NO_OF_ROBOTS_IN_FORMATION):
			for r in range(3):
				# set Leader Velocity
				leadr = [i for i in robots if isinstance(i, Leader)]
				# robots[f_graph.get('l')].setCmdVel(1, .01)
				leadr[0].setCmdVel(1.0, 0.0) #Subhasis : to change
				# dt = time.time() - time_start
				dt = .1
				# if r == f_graph.get('l'):
				if isinstance(robots[r], Leader):
					robots[r].update(robots[r].v, robots[r].w, dt)
					plot_leader.append(robots[r].pose.x)
					plot_leader.append(robots[r].pose.y)
					# print robots[r].pose.x
				else:
					robots[r].setCmdVel(dist, C.Phij[r], dt) #Subhasis : to change
					robots[r].update(robots[r].v, robots[r].w, dt)
					plot_followers[r].append(robots[r].pose.x)
					plot_followers[r].append(robots[r].pose.y)
					# print robots[r].pose.y
				time_start = time.time()
		except KeyboardInterrupt:
			print "Code ends"
			sys.exit(1)

	plt.axis('equal')
	A, B = plot_leader[::2], plot_leader[1::2]		
	plt.plot(A, B, 'r')
	colors = [k for k,v in pltc.cnames.items()]
	for i in range(1, len(plot_followers)):
		x, y = plot_followers[i][::2], plot_followers[i][1::2]
		plt.plot(x, y, colors[i])
	plt.show()

	sys.exit(1)


if __name__ == '__main__':
	main()

















#To replace a robot
#robots[1], robots[0] = Pibot.swap(robots[1], robots[0])


# def fit_discharge(Dis): 
#     b =       3.409 
#     c =       39.55 
#     d =   -0.002653  
#     e =    -0.03203 
#     f =  -8.112e-08
#     dis = (a+c*Dis+e*Dis^2)/(1 + b*Dis + d*Dis^2 + f*Dis^3);
