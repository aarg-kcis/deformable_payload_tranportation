#!/usr/bin/python
from __future__ import division

import sys
sys.path.append("../")
import time
from math import radians, degrees, pi
from matplotlib import pyplot as plt
import matplotlib.colors as pltc
import numpy as np
from formation import Pibot, Leader, Follower
from pose import Pose
import constants as C
import pickle
import myFunc72 as mfm
from shapely.geometry import LineString
from six_bots_in_formation import generate_bot_paths
import F2_generate_vw as F2

def draw_bot(state, bot_rad, color):
	plt.plot([state.x , state.x + bot_rad * np.cos(state.theta)], [state.y , state.y + bot_rad * np.sin(state.theta)],  color)
	plt.plot([state.x + bot_rad * np.cos(state.theta - np.pi/2), state.x + bot_rad * np.cos(state.theta + np.pi/2)], [state.y + bot_rad * np.sin(state.theta - np.pi/2), state.y + bot_rad * np.sin(state.theta + np.pi/2)], color)


def get_Lij(leaderPose, botAngle, bot_line_string, r):
	Bot_angle = leaderPose.theta + botAngle
	Bot_point_x = leaderPose.x + mfm.idealDistOfBotFromCentr * 5.5 * np.cos(Bot_angle)
	Bot_point_y = leaderPose.y + mfm.idealDistOfBotFromCentr * 5.5 * np.sin(Bot_angle)
	Bot_point = LineString([[Bot_point_x, Bot_point_y], [leaderPose.x, leaderPose.y]])
	intersectionPoint = Bot_point.intersection(bot_line_string)
#	print intersectionPoint.type
#	if r == 6:
#		print Bot_point_x, Bot_point_y, leaderPose.x, leaderPose.y
#		plt.plot([Bot_point_x, leaderPose.x], [Bot_point_y,leaderPose.y], 'k')	
	if intersectionPoint.type == 'GeometryCollection':
		return False, 0 
	
	if intersectionPoint.type == 'MultiPoint':
		minDist = 1.5 * mfm.idealDistOfBotFromCentr 
		for j in range(len(intersectionPoint)):
	    		dist = mfm.distance_between_points(leaderPose.x, leaderPose.y, intersectionPoint[j].x, intersectionPoint[j].y)
			if dist < minDist:
				minDist = dist
				intersectionX, intersectionY = intersectionPoint[j].x, intersectionPoint[j].y
	elif intersectionPoint.type == 'Point':
		intersectionX, intersectionY = intersectionPoint.x, intersectionPoint.y

	return True, mfm.distance_between_points(leaderPose.x, leaderPose.y, intersectionX, intersectionY)

def main():
	# start time to calculate dt
	time_start = time.time()
	counter = 0
	robots = []

	fig, ax= plt.subplots()
	ax.axis('equal')


	noOfRobots = 72
	leftBotAngles = []
	rightBotAngles = []
	for l in range(int(noOfRobots/2)):
		leftBotAngles.append(np.pi/noOfRobots + 2*l * np.pi/noOfRobots)
		rightBotAngles.append(-(np.pi/noOfRobots + 2*l * np.pi/noOfRobots))

	print np.array(leftBotAngles) * 180/np.pi
        long_path, leftBotPaths, rightBotPaths = generate_bot_paths(leftBotAngles, rightBotAngles)
	print len(leftBotPaths), len(rightBotPaths)

	x0, y0, theta0 = long_path[2][0], long_path[2][1], mfm.angle_wrt_x(long_path[2], long_path[3])

	botAngles = [0]
	bot_line_string = [0]
	for i in range(len(leftBotAngles)):
		botAngles.append(leftBotAngles[i])
		bot_line_string.append(LineString(leftBotPaths[i].tolist()))
	for i in range(len(rightBotAngles)):
		botAngles.append(rightBotAngles[i])
		bot_line_string.append(LineString(rightBotPaths[i].tolist()))


	# plot variables
	plot_leader = []
	plot_followers = [[] for i in range(C.NO_OF_ROBOTS_IN_FORMATION)]

	# create instances for all the robots (ID = 0 as leader and others as followers initially)
	robots.append(Leader(0, Pose(x0, y0, theta0)))
#	for i in range(1, C.MAX_ROBOTS):
	for i in range(1, C.NO_OF_ROBOTS_IN_FORMATION):
		robots.append(Follower(int(i), Pose(x0 + (mfm.idealDistOfBotFromCentr - mfm.bot_rad) * np.cos(theta0 + botAngles[i]), y0 + (mfm.idealDistOfBotFromCentr - mfm.bot_rad) * np.sin(theta0 + botAngles[i]), theta0), robots[0]))


	# Formation Graph (out of 20 robots)
	f_graph = {'l':1, 'f':range(1, 50)}

	HARD_COLLISION = 0
	SAFE_COLLISION = 1
	NO_COLLISION = 2
	DANGER_ZONE = 3
	target_speed = 0.1
	dt = 0.1
	cx, cy = long_path.T
	state = F2.State(x= cx[2], y=cy[2], yaw=mfm.angle_wrt_x(long_path[2], long_path[3]), v=0.0)
	target_ind = F2.calc_target_index(state, cx, cy)
	allDist = [0] * C.NO_OF_ROBOTS_IN_FORMATION
	obstPose = np.array([25, 10, 1.0])
	obstRad = 0.5
	collisionFlag = NO_COLLISION
	obstSpeed = 0.1
	safeDist = 0.2
	collisionChkDist = 0.2 #middle circle
	moveBack = False
	maxSpeed = 0.5
	normalSpeed = 0.1
	R1 = mfm.idealDistOfBotFromCentr + mfm.bot_rad
	R2 = R1 + collisionChkDist
	R3 = R2 + safeDist
	animation = False

	allVel = []
	allWom = []

	# run formation in the main loop
	while True:
		try:
#			if counter > 2501:
#				break
			counter  = counter + 1

#			raw_input()
			print counter
			plt.cla()
			if obstPose[0] > -5:
				obstPose[0] = obstPose[0] - (obstSpeed*dt)

			distOfObst = mfm.distance_between_points(obstPose[0], obstPose[1], robots[0].pose.x, robots[0].pose.y)
			obstAngle = mfm.angle_wrt_x((robots[0].pose.x, robots[0].pose.y), (obstPose[0], obstPose[1]))
			if abs(obstAngle - robots[0].pose.theta) <= np.pi/2:
				moveBack = True
			else:
				moveBack = False

			if distOfObst <= (obstRad + R1):
				collisionFlag = HARD_COLLISION
				c1 = 'r'
				c2 = 'r'
				c3 = 'r'
			elif distOfObst <= (obstRad + R2) and distOfObst > (obstRad + R1):
				collisionFlag = SAFE_COLLISION
				c1 = 'g'
				c2 = 'r'
				c3 = 'r'
			elif distOfObst > (obstRad + R2) and distOfObst <= (obstRad + R3) :
				collisionFlag = DANGER_ZONE
				c1 = 'g'
				c2 = 'g'
				c3 = 'r'
			else:
				collisionFlag = NO_COLLISION
				c1 = 'g'
				c2 = 'g'
				c3 = 'g'






			for r in range(C.NO_OF_ROBOTS_IN_FORMATION):
				# set Leader Velocity
				leadr = [i for i in robots if isinstance(i, Leader)]
				# robots[f_graph.get('l')].setCmdVel(1, .01)

				if collisionFlag ==NO_COLLISION  and r==0: #if no collision... Go straight...
					print "No collision"
					target_speed = normalSpeed
					ai = F2.PIDControl(target_speed, state.v)
					di, target_ind , delX, Lf = F2.pure_pursuit_control(state, cx, cy, target_ind)
					state = F2.update(state, ai, di, delX, Lf )
					print target_speed
				if collisionFlag == SAFE_COLLISION and r==0:
					print "safe collision..."
					if moveBack:
						target_speed = -(normalSpeed + (maxSpeed-normalSpeed) * (R2 + obstRad - distOfObst) / (R2 - R1))
						ai = F2.PIDControl(target_speed, state.v)
						di, target_ind, delX, Lf = F2.pure_pursuit_control(state, np.flip(cx, 0), np.flip(cy, 0), target_ind)
						state = F2.update(state, ai, di, delX, Lf )
					else:
						target_speed = normalSpeed + (maxSpeed-normalSpeed) * (R2 + obstRad - distOfObst) / (R2 - R1)
						ai = F2.PIDControl(target_speed, state.v)
						di, target_ind, delX, Lf = F2.pure_pursuit_control(state, cx, cy, target_ind)
						state = F2.update(state, ai, di, delX, Lf )
					print target_speed
				if collisionFlag == DANGER_ZONE  and r==0:
					if moveBack:
						print "Danger zone... Move Back..."
						target_speed = 0.0
						ai = F2.PIDControl(target_speed, state.v)
						di, target_ind , delX, Lf = F2.pure_pursuit_control(state, np.flip(cx, 0), np.flip(cy, 0), target_ind)
						#di, target_ind = F2.pure_pursuit_control(state, cx, cy, target_ind)
						state = F2.update(state, ai, di, delX, Lf )
					else:
						print "Danger zone... Move Forward..."
						target_speed = normalSpeed + (maxSpeed-normalSpeed) * (R3 + obstRad - distOfObst) / (R3 - R2)
						ai = F2.PIDControl(target_speed, state.v)
						#di, target_ind = F2.pure_pursuit_control(state, np.flip(cx, 0), np.flip(cy, 0), target_ind)
						di, target_ind , delX, Lf = F2.pure_pursuit_control(state, cx, cy, target_ind)
						state = F2.update(state, ai, di, delX, Lf )
					print target_speed
				if collisionFlag == HARD_COLLISION and r==0:
					print "hard collisionnnnnnnnnn..."
					if moveBack:
						target_speed = -maxSpeed
						ai = F2.PIDControl(target_speed, state.v)
						di, target_ind , delX, Lf = F2.pure_pursuit_control(state, np.flip(cx, 0), np.flip(cy, 0), target_ind)
						state = F2.update(state, ai, di, delX, Lf )
					else:
						target_speed = maxSpeed
						ai = F2.PIDControl(target_speed, state.v)
						di, target_ind , delX, Lf = F2.pure_pursuit_control(state, cx, cy, target_ind)
						state = F2.update(state, ai, di, delX, Lf )
					print target_speed
				leadr[0].setCmdVel(state.v, state.w) #Subhasis : to change

				if isinstance(robots[r], Leader):

					robots[r].update(robots[r].v, robots[r].w, dt)
					plot_leader.append(robots[r].pose.x)
					plot_leader.append(robots[r].pose.y)
					# print robots[r].pose.x
				else:
					retVal, dist = get_Lij(robots[0].pose, botAngles[r], bot_line_string[r], r)	
					if retVal:
						allDist[r] = dist
					robots[r].setCmdVel(allDist[r], botAngles[r], dt) #Subhasis : to change


					if collisionFlag == SAFE_COLLISION or collisionFlag == DANGER_ZONE:
						if moveBack:
							robots[r].update_back(robots[r].v, robots[r].w, dt)
						else:
							robots[r].update(robots[r].v, robots[r].w, dt)
					else:
						robots[r].update(robots[r].v, robots[r].w, dt)

				time_start = time.time()
			for c in range(1, C.NO_OF_ROBOTS_IN_FORMATION):
				if c==C.NO_OF_ROBOTS_IN_FORMATION-1:
					cpp = 1
				else:
					cpp = c + 1
				if mfm.distance_between_points(robots[c].pose.x, robots[c].pose.y, robots[cpp].pose.x, robots[cpp].pose.y) < 2*mfm.bot_rad:
					print "collision between ", c, " and ", cpp 
		
			tempV, tempW = [], []
			for l in range(C.NO_OF_ROBOTS_IN_FORMATION):
				tempV.append(robots[l].v)
				tempW.append(robots[l].w)
			allVel.append(tempV)
			allWom.append(tempW)
			
		
#			if counter == 2 or counter == 315 or counter == 500 or counter==750 or counter==860 or counter==1024 or counter == 1225 or counter==1500:
			if counter%10 == 0:
				plt.axis('equal')
				A, B = plot_leader[::2], plot_leader[1::2]		
				plt.plot(A, B, 'g')
				draw_bot(robots[0].pose, mfm.bot_rad*2, 'k')
	
				for dr in range(1, C.NO_OF_ROBOTS_IN_FORMATION):
#					ax.add_patch(plt.Circle((robots[dr].pose.x, robots[dr].pose.y), mfm.bot_rad, color='k', fill = False))
					draw_bot(robots[dr].pose, mfm.bot_rad, 'k')
	
					
				ax.add_patch(plt.Circle((obstPose[0], obstPose[1]), obstRad, color='b', fill=False))
				ax.add_patch(plt.Circle((robots[0].pose.x, robots[0].pose.y),  mfm.idealDistOfBotFromCentr + mfm.bot_rad, color=c1, fill=False))
				ax.add_patch(plt.Circle((robots[0].pose.x, robots[0].pose.y),  mfm.idealDistOfBotFromCentr + mfm.bot_rad + safeDist, color=c2, fill=False))
				ax.add_patch(plt.Circle((robots[0].pose.x, robots[0].pose.y),  mfm.idealDistOfBotFromCentr + mfm.bot_rad + safeDist + collisionChkDist, color=c3, fill=False))
				for obstacles in mfm.circularObstacles:
					ax.add_patch(plt.Circle((obstacles[0], obstacles[1]), obstacles[2], color='r'))
			
				long_pathX, long_pathY = long_path.T
				plt.plot(long_pathX, long_pathY, 'g')
				plt.pause(0.001)
			
		except KeyboardInterrupt:
			print "Code ends"
			sys.exit(1)


	print "Done Simulation..."
	plt.show()
	allVel = np.array(allVel)
	allWom = np.array(allWom)
	np.savetxt('./allTextFiles/allVel.txt', allVel)	
	np.savetxt('./allTextFiles/allWom.txt', allWom)	

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
