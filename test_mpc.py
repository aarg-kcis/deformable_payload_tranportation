from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og
from math import sqrt
from sys import argv
import argparse
import myFuncCircle as mfm
import ompl_demo as odm
import matplotlib.pyplot as plt
from scipy.ndimage.filters import gaussian_filter as scipy_gaussian
import numpy as np
import math
from scipy.ndimage.filters import gaussian_filter as scipy_gaussian
import scipy.ndimage
from scipy.interpolate import interp1d
import shapely.geometry as geom
import F2_generate_vw as generateVW
import F3_get_bot_positions as get_bot_positions
import F4_run_formation as rf
import cvxpy
import time
class State:
	def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
		self.x = x
		self.y = y
		self.yaw = yaw

def update(state, v, w):

	state.yaw = state.yaw + w * dt
	state.x = state.x + v * math.cos(state.yaw) * dt
	state.y = state.y + v * math.sin(state.yaw) * dt
	
	return state

def get_linear_model_matrix(v, theta, w):
	A = np.matrix(np.zeros((NX, NX)))
	A[0, 0] = 1.0
	A[1, 1] = 1.0
	A[2, 2] = 1.0
	A[0, 2] = -v * math.sin(theta) * dt
	A[1, 2] =  v * math.cos(theta) * dt
	
	B = np.matrix(np.zeros((NX, NU)))
	B[0, 0] = math.cos(theta) * dt
	B[1, 0] = math.sin(theta) * dt
	B[2, 1] = dt
	
	C = np.zeros(NX)
	C[0] = v * math.cos(theta)
	C[1] = v * math.sin(theta)
	C[2] = w
	
	return A, B, C

dt = 0.1
NX = 3
NU = 2
T  = 20
MAX_SPEED = 1.5
MIN_SPEED = -1.5
MAX_OMEGA = 1.5
MIN_OMEGA = -1.5
R = np.diag([1.0, 1.0])  # input cost matrix
Rd = np.diag([1.0, 1.0])  # input difference cost matrix
Q = np.diag([1.0, 1.0, 1.0]) 
Qf = Q

long_path = np.loadtxt('long_path.txt')
dist = mfm.distance_between_points(long_path[1][0], long_path[1][1], long_path[10][0], long_path[10][1])
temp_t = dist/MAX_SPEED
T =20# int(math.ceil(temp_t / dt)) + 1
print "T: ", T

#state_init = State(x = long_path[1][0], y = long_path[1][1], yaw = mfm.angle_wrt_x(long_path[1], long_path[2]))
#state_final = State(x = long_path[10][0], y = long_path[10][1], yaw = mfm.angle_wrt_x(long_path[10], long_path[11]))

state_init = State(x = 1, y = 1, yaw = np.pi/6)
state_final = State(x = 2, y = 2, yaw = 5*np.pi/6)

x0 = [state_init.x, state_init.y, state_init.yaw]
xF = [state_final.x, state_final.y, state_final.yaw]
u_guess = 0.1 * np.ones((NU, T)) # changing this makes difference

xref = np.zeros((NX, T + 1))
xbar = np.zeros((NX, T + 1))

x = cvxpy.Variable((NX, T + 1))
u = cvxpy.Variable((NU, T))

#t_init = time.time()
for count in range(10):
	cost = 0.0
	constraints = []

	#u_guess[:, 0] = 0.0
#	print "u_guess: ", u_guess.shape
#	print u_guess[0, :]
	xbar[:,0] = x0
	temp_state = State(x = x0[0], y = x0[1], yaw = x0[2])
	for i in range(1, xbar.shape[1]):
		temp_state = update(temp_state, u_guess[0][i-1], u_guess[1][i-1])
		xbar[:, i] = [temp_state.x, temp_state.y, temp_state.yaw]

	for t in range(T):
	
		cost += cvxpy.quad_form(u[:, t], R)
		cost += cvxpy.quad_form(xF - x[:, t], Q)
		
		A, B, C = get_linear_model_matrix(u_guess[0, t], xbar[2, t], u_guess[1, t]) #check here
		constraints += [x[:, t + 1] == A * x[:, t] + B * u[:, t] + C]   #removing C works for multi iteration case
	
		if t > 0:
			constraints += [u[0, t] <= u[0, t-1] + 0.05]
			constraints += [u[0, t] >= u[0, t-1] - 0.05]

			constraints += [u[1, t] <= u[1, t-1] + 0.5]
			constraints += [u[1, t] >= u[1, t-1] - 0.5]
	
	
	#	constraints += [u[0, :] <= MAX_SPEED]
		
	#	if t < (T - 1):
	#		cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
	#		constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <= MAX_DSTEER * DT]
	
	cost += cvxpy.quad_form(xF - x[:, T], Qf)
	
	constraints += [x[:, 0] == x0]
	constraints += [x[:, T] == xF]
#	constraints += [x[0, 3] == [long_path[3][0]]]
#	constraints += [x[1, 3] == [long_path[3][1]]]

	constraints += [u[0, :] <= MAX_SPEED]
	constraints += [u[0, :] >= MIN_SPEED]
	constraints += [u[1, :] <= MAX_OMEGA]
	constraints += [u[1, :] >= MIN_OMEGA]
	constraints += [u[:, 0] == 0]

	
	prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
	prob.solve()
	print prob.status
	
#print "time taken: ", time.time() - t_init	
	if prob.status != 'infeasible':
		u_guess = u.value
		final_point = [x.value[0, -1], x.value[0, -1]]

		print x.value[0, :]
		print x.value[1, :]
		print x.value[2, :] * 180/np.pi
		
		print "velocities: "
		print u.value[0, :]
		print "w: "
		print u.value[1, :]
		
		plotX = x.value[0, :]
		plotY = x.value[1, :]
		plt.cla()
		plt.axis("equal")
		for s in range(x.shape[1]):
			plot_state = State(x = x.value[0][s], y = x.value[1][s], yaw = x.value[2][s])
	                rf.draw_bot(plot_state, 'r', mfm.bot_rad/10)

		plt.plot(plotX, plotY, '-')
		plt.pause(0.2)
plt.show()
print "Error: ", mfm.distance_between_points(final_point[0], final_point[1], state_final.x, state_final.y)























