import numpy as np
import math
import matplotlib.pyplot as plt
import myFuncMulti as mfm
import pickle
from shapely.geometry import LineString
import myFuncMulti as mfm

dt = 0.1

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw


def update(state, v, w):

    state.x = state.x + v * math.cos(state.yaw + w * dt) * dt
    state.y = state.y + v * math.sin(state.yaw + w * dt) * dt
    state.yaw = state.yaw + w * dt

    return state

def get_bot_positions(long_path, Bot_path, v, w, botAngle, distOfBotFromCentr):
    cx, cy = long_path.T
    bx, by = Bot_path.T

    state = State(x= cx[2], y=cy[2], yaw=mfm.angle_wrt_x(long_path[2], long_path[3]))
    long_path_new = []   
 
    Bot_positions = []
    bot_dist = []
    Bot_line_string = LineString(Bot_path.tolist())
    for i in range(len(v)):
#        print state.x, state.y, v[i], w[i]
#        raw_input()
        Bot_angle = state.yaw + botAngle
        Bot_point_x = state.x + distOfBotFromCentr * 1.5 * np.cos(Bot_angle)
        Bot_point_y = state.y + distOfBotFromCentr * 1.5 * np.sin(Bot_angle)
        Bot_point = LineString([[Bot_point_x, Bot_point_y], [state.x, state.y]])
        intersectionPoint = Bot_point.intersection(Bot_line_string)
        if intersectionPoint.type == 'MultiPoint':
            minDist = 1.5 * distOfBotFromCentr
            for j in range(len(intersectionPoint)):
                dist = mfm.distance_between_points(state.x, state.y, intersectionPoint[j].x, intersectionPoint[j].y)
                if dist < minDist:
                    minDist = dist
                    intersectionX, intersectionY = intersectionPoint[j].x, intersectionPoint[j].y
 

    	elif intersectionPoint.type == 'Point':
            intersectionX, intersectionY = intersectionPoint.x, intersectionPoint.y
        else:
             pass
#             print "Unknown Intersection"
        Bot_positions.append([intersectionX, intersectionY])
        bot_dist.append(mfm.distance_between_points(state.x, state.y, intersectionX, intersectionY))
        long_path_new.append([state.x, state.y])
#        plt.cla()
#        plt.plot([state.x], [state.y], 'x')
#        plt.plot(bx, by)
#        plt.plot(cx, cy)
#        plt.plot([intersectionX],[intersectionY],'.')
#        plt.plot([state.x, Bot_point_x],[state.y, Bot_point_y])
#        plt.pause(0.001)
        state = update(state, v[i], w[i])
    print "inside get_bot_position, type of position: ", type(Bot_positions)
    return Bot_positions, np.array(bot_dist), np.array(long_path_new)

def main():
    long_path = np.loadtxt('./allPaths/long_path.txt')
    Bot_path = np.loadtxt('./allPaths/botl_path.txt')
    cx, cy = long_path.T
    
    state = State(x= cx[2], y=cy[2], yaw=mfm.angle_wrt_x(long_path[2], long_path[3]))
    
    with open('./allPaths/v.txt', 'rb') as fv:
        v = pickle.load(fv)
    with open('./allPaths/w.txt', 'rb') as fw:
        w = pickle.load(fw)
    
    print long_path.shape
    print len(v)
    Bot_Lij = []
    Bot_Thetaij = []
    Bot_positions = []
    Bot_line_string = LineString(Bot_path.tolist())
    for i in range(len(v)):
        Bot_angle = state.yaw + botAngle
	print botAngle
        Bot_point_x = state.x + 1 * np.cos(Bot_angle)
        Bot_point_y = state.y + 1 * np.sin(Bot_angle)
        Bot_point = LineString([[Bot_point_x, Bot_point_y], [state.x, state.y]])
        intersectionPoint = Bot_point.intersection(Bot_line_string)
        Bot_positions.append([intersectionPoint.x, intersectionPoint.y])
        Bot_Lij.append(mfm.distance_between_points(intersectionPoint.x, intersectionPoint.y, state.x, state.y)) 
        Bot_Thetaij.append(Bot_angle)
        state = update(state, v[i], w[i])
    #    plt.plot([state.x],[state.y], 'r.')
    #    plt.pause(0.001)
    with open('./allTextFiles/rightBot_Lij.txt', 'wb') as fLij:
        pickle.dump(leftBot_Lij, fLij)
    with open('./allTextFiles/rightBot_Thetaij.txt', 'wb') as fThetaij:
        pickle.dump(leftBot_Thetaij, fThetaij)
    with open('./allTextFiles/rightBot_positions.txt', 'wb') as Lpos:
        pickle.dump(leftBot_positions, Lpos)
     
    print len(v), len(leftBot_Thetaij)
    print max(leftBot_Lij)
    plt.plot(cx, cy, 'g')
     
    plt.show()
    
    print "showing Path"
    state = State(x= cx[2], y=cy[2], yaw=mfm.angle_wrt_x(long_path[2], long_path[3]))
    for i in range(len(v)):
        plt.plot(state.x + leftBot_Lij[i] * np.cos(leftBot_Thetaij[i]), state.y + leftBot_Lij[i] * np.sin(leftBot_Thetaij[i]), 'r.')
        state  = update(state, v[i], w[i])
    
    leftBot_pathX, leftBot_pathY = leftBot_path.T
    plt.plot(leftBot_pathX, leftBot_pathY, 'g.')
    
    plt.show()











