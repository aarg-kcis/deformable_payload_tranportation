from __future__ import division
from numpy import radians, degrees, pi

# Max iterations
MAX_ITER = 5000

# Robot Hardware parameters
L = 0.12 # Wheel base = 2 * L (in meters)
R = 0.035 # radius of wheels (in meters)
d = 0.1 # in meters

# Formation gains
k1 = 1.5
k2 = 4.0
k3 = 0.025
A = 10.0
B = 1.0
D = 1.0

# Formation parameters for payload transport
STARTING_LOCATION = [0, 0]
MAX_ROBOTS = 10
LOAD_PER_ROBOT = 5.0 # in Kgs
PAYLOAD_WEIGHT = 20.0 # in Kgs
MAX_HUBS = 3
STARTING_HUB = 1
NO_OF_ROBOTS_IN_FORMATION = 7#int(PAYLOAD_WEIGHT/LOAD_PER_ROBOT) + 1

# Battery Constants
BATTERY_MAX_CAPACITY = 1200.0 # in mAH
MAX_BATTERY_STATES = 10 #(10, 20, 30, 40, 50, 60, 70, 80, 90, 100)
HUB_TO_HUB_DISCHARGE_CONSTANT = 150

# test case, to be removed if dealing with dynamics
CHARGE_RATE = 0.1
DISCHARGE_RATE = -0.1

# optimization Constants
ALPHA 	= 0.999
BETA 	= 1 - ALPHA

# Pose to maintain from leader. Leader is always the 1st robot in formation
Lij = 10 # in meters
Phij = []
phi_eq = radians(360/(NO_OF_ROBOTS_IN_FORMATION-1)) # angle to maintain from leader, eg: 5 robots in formation, 1 leader and 4 followers in at 90 degrees each (360/4)
Phij.append(radians(90)) # leader Orientation
for i in range(1, NO_OF_ROBOTS_IN_FORMATION):
	phi = Phij[0] + i * phi_eq
	if phi > pi:
		phi = phi - 2*pi
	if phi < -pi:
		phi = phi + 2*pi
	Phij.append(phi)

