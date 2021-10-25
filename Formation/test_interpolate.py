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

#pathFound, path = odm.plan(mfm.runTime, mfm.plannerType, mfm.objectiveType, mfm.fname, mfm.bound, mfm.start_pt, mfm.goal_pt)
#
#if pathFound:
#	pathList = mfm.getPathPointAsList(path)
#	pathList = np.array(pathList) 		# pathList is numpy array now
#else:
#	print "Path not found..."
#
#print pathList.shape
#print pathList

pathList = np.array([[0,0], [5,0], [10,0]])
long_path = np.array([[0,0]])
print long_path.shape
f = interp1d(pathList[:,0], pathList[:,1], kind='quadratic')
x = np.linspace(0, 10, num=20, endpoint=True)
y = f(x)
for i in range(x.shape[0]):
	long_path = np.append(long_path, [[x[i], y[i]]], axis=0)
long_path = np.delete(long_path, 0, axis=0)
x, y = long_path.T
plt.plot(x, y, '.r')

pathList = np.array([[0,1], [5,1], [10,1]])
long_path = np.array([[0,0]])
print long_path.shape
f = interp1d(pathList[:,0], pathList[:,1], kind='quadratic')
x = np.linspace(0, 10, num=20, endpoint=True)
y = f(x)
for i in range(x.shape[0]):
	long_path = np.append(long_path, [[x[i], y[i]]], axis=0)
long_path = np.delete(long_path, 0, axis=0)
x, y = long_path.T
plt.plot(x, y, '.r')

pathList = np.array([[0,2], [5,2], [10,2]])
long_path = np.array([[0,0]])
print long_path.shape
f = interp1d(pathList[:,0], pathList[:,1], kind='quadratic')
x = np.linspace(0, 10, num=20, endpoint=True)
y = f(x)
for i in range(x.shape[0]):
	long_path = np.append(long_path, [[x[i], y[i]]], axis=0)
long_path = np.delete(long_path, 0, axis=0)
x, y = long_path.T
plt.plot(x, y, '.r')

plt.plot([0],[-1])
plt.show()
exit()

long_path = np.array([[0,0]])
print long_path.shape
f = interp1d(pathList[:,0], pathList[:,1], kind='linear')
x = np.linspace(0, 10, num=500, endpoint=True)
y = f(x)
for i in range(x.shape[0]):
	long_path = np.append(long_path, [[x[i], y[i]]], axis=0)
long_path = np.delete(long_path, 0, axis=0)

print long_path.shape
x, y = long_path.T
plt.plot(x, y, '.b')
plt.show()
