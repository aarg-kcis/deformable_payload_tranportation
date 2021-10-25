from math import sqrt
import numpy as np
import math
from scipy.ndimage.filters import gaussian_filter as scipy_gaussian
import scipy.ndimage
import matplotlib.pyplot as plt
# Set parameters for planner. Check ompl_demo.py for more available parameters
runTime = 3
plannerType = 'RRTstar' #'BFMTstar'#'BITstar' 'RRTstart'
objectiveType = 'PathLength'
fname= None

# Give start and goal points
bound = (0.0, 10.0) #Basically the step size
start_pt = (0,0)
goal_pt = (10,10)
circularObstacles = [(3,1,1), (3,4,1), (8,2,2), (3,8,2), (6.5, 6.0,  0.5), (9,7,1), (6.5, 7.5,  0.5) ,(8, 5.5,  0.5)]
bot_rad = 0.1
idealDistOfBotFromCentr = 1 
minDistOfClearance = bot_rad*2  + bot_rad #Change it for clearance during path planning
resolutionOfPathPoints = bot_rad/16

def distance_between_points(x1,y1,x2,y2):
	return sqrt((x1-x2)**2 + (y1-y2)**2)

def angle_wrt_x((x1,y1), (x2,y2)):
	return np.arctan2(y2-y1,x2-x1)

def radius_from_3Points((x1, y1), (x2,y2), (x3,y3)):
	x, y, z = complex(x1,y1), complex(x2, y2), complex(x3, y3)
	w = z-x
	w /= y-x
	c = (x-y)*(w-abs(w)**2)/2j/w.imag-x
	return abs(c + x)

def myClearance(x, y):

	for obstacles in circularObstacles:
		val = sqrt((x-obstacles[0])**2 + (y-obstacles[1])**2) - obstacles[2] - minDistOfClearance
		if val > 0:
			success = True
		else:
			success = False
			break;
	

	if (success): 
		return 1
	else:
		return -1


def getPathPointAsList(path):
	# print path
	string = [s.strip() for s in path.splitlines()]
	# print string
	pathList = []
	for i in string:
		if i != '':
			print i
			pathList.append(i.split(" "))
	# print pathList
	for j in pathList:
		j[0] = float(j[0])
		j[1] = float(j[1])
	print pathList
	return pathList




def getInterpolatedPath(pathList):
	long_path = np.copy([pathList[0]])
	resolution = resolutionOfPathPoints	
	for i in range(1,pathList.shape[0]):
		temp_path = np.copy([pathList[i-1]])
		theta = np.arctan2(pathList[i][1]-pathList[i-1][1], pathList[i][0]-pathList[i-1][0])    
		dist = distance_between_points(pathList[i][0], pathList[i][1], pathList[i-1][0], pathList[i-1][1])
		temp_dist = distance_between_points(temp_path[0][0], temp_path[0][1], pathList[i-1][0], pathList[i-1][1])
		
		while(temp_dist < dist-resolution):
		        temp_path = np.append(temp_path, [[temp_path[-1][0] + (resolution)*np.cos(theta), temp_path[-1][1] + (resolution)*np.sin(theta)]], axis = 0)
		        temp_dist = distance_between_points(temp_path[-1][0], temp_path[-1][1], temp_path[0][0], temp_path[0][1])
		
		long_path = np.append(long_path, temp_path, axis = 0)

	return long_path

def curvefit(pathList) :
	x = pathList[0]
	y = pathList[1]
	order = 4
	points = np.linspace(0,1,len(x))
	new_points = np.linspace(0,1,3*len(x))
	x_coeff = np.polyfit(points,x,order)
	y_coeff = np.polyfit(points,y,order)

	func_x = np.poly1d(x_coeff)
	func_y = np.poly1d(y_coeff)

	new_x = func_x(new_points)
	new_y = func_y(new_points)

	# pathList_new[0] = new_x
	# pathList_new[1] = new_y
	
	plt.figure(3)
	plt.plot(new_x,new_y, 'o')
	plt.figure(4)
	plt.plot(pathList[0],pathList[1],'x')




def intersectWithCircle((x, y), (x_prev, y_prev), (center_x, center_y), r, angle_in_formation): #angle_in_formation in radians
        if (x-x_prev) == 0:
                m = 0
        else:
                m = angle_in_formation + math.atan(float(y-y_prev)/float(x-x_prev))
        if m >= math.pi:
                m = m - math.pi
        m = math.tan(m)
        c = y - (m * x)
#       print "slope is" , m
#       print "c is", c
        p = center_x
        q = center_y
        A = m**2 + 1
        B = 2*(m*c - m*q -p)
        C = q**2 - r**2 + p**2 - 2*c*q + c**2

        discriminant = B**2 - 4*A*C
        if discriminant < 0:
                return False, ()
        if discriminant == 0:
                x_new = -(B/(2*A))
                y_new = m*x_new + c
                return True, (x_new, y_new)
        if discriminant > 0:
                x1 = ((-B) + math.sqrt(discriminant))/(2*A)
                y1 = m*x1 + c
                x2 = ((-B) - math.sqrt(discriminant))/(2*A)
                y2 = m*x2 + c

                if (((x1 - x)**2 + (y1 - y)**2) < ((x2 - x)**2 + (y2 - y)**2)):
                        return True, (x1, y1)
                else:
                        return True, (x2, y2)



def circleOnLeft(origin, point, circle):
	a = np.array([point[0]-origin[0], point[1]-origin[1]])
	b = np.array([circle[0]-origin[0], circle[1]-origin[1]])
	c = np.cross(a,b)
	if c>0:
		return True
	else:
		return False




	#########################################################################################
	
def right_bot_dist_generator(long_path, bot_angle):
	bot2_dist = np.array([ idealDistOfBotFromCentr-bot_rad])   #bot_rad is given 0.13 and  idealDistOfBotFromCentr is the distance between bot and center of the formation, change  idealDistOfBotFromCentr to root(2)* idealDistOfBotFromCentr for the bot which is at 45 degree angle
	for i in range(1, long_path.shape[0]):
	#for i in range(40, 51):
		final_dist = 10 # Some max value from which distance to intersection with obstacle has to be less.
		final_intersection = ()
		for obstacles in circularObstacles:
			if (not circleOnLeft(long_path[i-1], long_path[i], obstacles)):
				ret, intersection = intersectWithCircle(tuple(long_path[i]), tuple(long_path[i-1]), (obstacles[0], obstacles[1]), obstacles[2], bot_angle)
				if ret == True:
					current_dist = distance_between_points(intersection[0], intersection[1], long_path[i][0], long_path[i][1]) 
					if current_dist<final_dist:
						final_dist = current_dist
	
		current_theta = math.atan2(float(long_path[i][1] - long_path[i-1][1]), float(long_path[i][0] - long_path[i-1][0]))
	        bot2_theta = current_theta + ( bot_angle)
	
		if final_dist >  idealDistOfBotFromCentr:
			bot2_dist = np.append(bot2_dist, [ idealDistOfBotFromCentr-bot_rad ], axis = 0)
		elif final_dist > 2*bot_rad+bot_rad and final_dist <= idealDistOfBotFromCentr :
			bot2_dist = np.append(bot2_dist, [(((final_dist-minDistOfClearance)/2 + 2*bot_rad) )], axis = 0) #Subhasis: Changed here
												#2*bot_rad is rho_I_min here... customised for six bots
		else:
			bot2_dist = np.append(bot2_dist, [((final_dist-bot_rad) )], axis = 0) #Subhasis: Changed here
			print bot2_dist
			print "Collision detected... Run again...."

#	np.savetxt('bot2_dist_o.txt', bot2_dist)
	bot2_dist_initial = np.copy(bot2_dist)
	bot2_dist = scipy.ndimage.minimum_filter1d(bot2_dist,size=50)
	bot2_dist = scipy_gaussian(bot2_dist,sigma=20)
	bot2_dist = np.minimum(bot2_dist, bot2_dist_initial)
	#bot2_dist = scipy_gaussian(bot2_dist,sigma=2)
#	np.savetxt('bot2_dist_s.txt', bot2_dist)
	return bot2_dist
	
	
	##########################################################################################


def left_bot_dist_generator(long_path, bot_angle):
	bot1_dist = np.array([ idealDistOfBotFromCentr-bot_rad])
	for i in range(1, long_path.shape[0]):
	#for i in range(40, 51):
		final_dist = 10
		final_intersection = ()
		for obstacles in circularObstacles:
			if (circleOnLeft(long_path[i-1], long_path[i], obstacles)):
				ret, intersection = intersectWithCircle(tuple(long_path[i]), tuple(long_path[i-1]), (obstacles[0], obstacles[1]), obstacles[2], bot_angle)
				if ret == True:
					current_dist = distance_between_points(intersection[0], intersection[1], long_path[i][0], long_path[i][1]) 
					if current_dist<final_dist:
						final_dist = current_dist
	
		current_theta = math.atan2(float(long_path[i][1] - long_path[i-1][1]), float(long_path[i][0] - long_path[i-1][0]))
	        bot1_theta = current_theta + bot_angle
	
		if final_dist >  idealDistOfBotFromCentr:
			bot1_dist = np.append(bot1_dist, [( idealDistOfBotFromCentr-bot_rad) ], axis = 0)
		elif final_dist > 2*bot_rad+bot_rad and final_dist <= idealDistOfBotFromCentr :
			bot1_dist = np.append(bot1_dist, [(((final_dist-minDistOfClearance)/2 + 2*bot_rad) )], axis = 0) #Subhasis: Changed here
		else:
			bot1_dist = np.append(bot1_dist, [((final_dist-bot_rad)  )], axis = 0)
			print bot1_dist
			print "Collision detected... Run again...."
	
	bot1_dist_initial = np.copy(bot1_dist)
	bot1_dist = scipy.ndimage.minimum_filter1d(bot1_dist,size=50)
	bot1_dist = scipy_gaussian(bot1_dist,sigma=20)
	bot1_dist = np.minimum(bot1_dist, bot1_dist_initial)
	#bot1_dist = scipy_gaussian(bot1_dist,sigma=2)
	return bot1_dist
	
#################################################################################################

def path_generation_in_pair(long_path, left_dist, right_dist, leftBot_angle, rightBot_angle):
	leftBot_forward_path = np.array([[0,0]])
	rightBot_forward_path = np.array([[0,0]])
	success = True
	if (leftBot_angle != math.pi/2) or (rightBot_angle != -math.pi/2):
		print "Pair wise path is generated only for bots at 90 degrees from enter..."
		success = False
	for i in range(1, long_path.shape[0]):
		if (not success):
			break
		if left_dist[i] < bot_rad and right_dist[i] < bot_rad:
			print "No proper path generated by OMPL..."
			print "Check long path point", i
		        success = False
			break
		if left_dist[i] < bot_rad:
			right_dist[i] = right_dist[i] + bot_rad - left_dist[i]
		if right_dist[i] < bot_rad:
			left_dist[i] = left_dist[i] + bot_rad - right_dist[i]
		
		current_theta = math.atan2(float(long_path[i][1] - long_path[i-1][1]), float(long_path[i][0] - long_path[i-1][0]))
		
		leftBot_theta = current_theta + leftBot_angle
		next_pt =np.array([[long_path[i][0] + (left_dist[i]) * np.cos(leftBot_theta), long_path[i][1] + (left_dist[i]) * np.sin(leftBot_theta)]])
		leftBot_forward_path = np.append(leftBot_forward_path, next_pt, axis = 0)

		rightBot_theta = current_theta + (rightBot_angle)
		next_pt =np.array([[long_path[i][0] + (right_dist[i]) * np.cos(rightBot_theta), long_path[i][1] + (right_dist[i]) * np.sin(rightBot_theta)]])
		rightBot_forward_path = np.append(rightBot_forward_path, next_pt, axis = 0)

	return success, leftBot_forward_path, rightBot_forward_path


#######################################################################################################################

def bot_path_generator(long_path, bot_dist, bot_angle):
	bot_path = np.array([[0,0]])
	for i in range(1, long_path.shape[0]):
		
		current_theta = math.atan2(float(long_path[i][1] - long_path[i-1][1]), float(long_path[i][0] - long_path[i-1][0]))
		
		bot_theta = current_theta + bot_angle
		next_pt =np.array([[long_path[i][0] + (bot_dist[i]) * np.cos(bot_theta), long_path[i][1] + (bot_dist[i]) * np.sin(bot_theta)]])
		bot_path = np.append(bot_path, next_pt, axis = 0)

	return bot_path

