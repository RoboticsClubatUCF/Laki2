# Andrew Schroeder
# 9/13/2018

# script to find the path length traced out by a one, two, or three dimensional parametric function
# function is given by < x(t), y(t), z(t) >
# path length is found by integrating the following function with respect to t from t1 to t2:
# sqrt{ (dx/dt)^2 + (dy/dt)^2 + (dz/dt)^2 }

import math
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.misc import derivative
import scipy.integrate as integrate
# import matplotlib.pyplot as plt

wpx = [250, 100, 1000, 1050, 800, 750]
wpy = [600, 375, 390, 650, 650, 200]
wpz = [130, 120, 150, 110, 200, 105]

n = len(wpx)
tvals = np.arange(n)
# t = [0, 1, 2, ...]

csx = CubicSpline(tvals, wpx)
csy = CubicSpline(tvals, wpy)
csz = CubicSpline(tvals, wpz)
# print csx.c
# print csy.c
# print csz.c, "\n"

totalLength = 0

for i in range(len(tvals) - 1):

	# tX is a list of the coefficients of the equation x = At^3 + Bt^2 + Ct + D
	# tY is a list of the coefficients of the equation y = at^3 + bt^2 + ct + d
	# tZ is a list of the coefficients of the equation z = aat^3 + bbt^2 + cct + dd

	tX = [csx.c[0][i], csx.c[1][i], csx.c[2][i], csx.c[3][i]]
	tY = [csy.c[0][i], csy.c[1][i], csy.c[2][i], csy.c[3][i]]
	tZ = [csz.c[0][i], csz.c[1][i], csz.c[2][i], csz.c[3][i]]
	
	# define the parametric functions fx(t), fy(t), and fz(t).

	def fx(t):
		return tX[0]*t**3 + tX[1]*t**2 + tX[2]*t + tX[3]
	
	def fy(t): 
		return tY[0]*t**3 + tY[1]*t**2 + tY[2]*t + tY[3]

	def fz(t): 
		return tZ[0]*t**3 + tZ[1]*t**2 + tZ[2]*t + tZ[3]
	
	
	# Integrate the square root of the sum of dX^2, dY^2, dZ^2 with respect to t 
	#	from t = i to t = i + 1
	length = integrate.quad(lambda t: math.sqrt(derivative(fx,t)**2 + derivative(fy,t)**2 + derivative(fz,t)**2), tvals[i], tvals[i + 1])
	length = length[0]


	print "The length of segment t = ", tvals[i], " to t = ", tvals[i + 1], " is ", length, "\n"

	# add the segment length to the total path length

	totalLength += length

return totalLength

print "The total length from t = 0 to t = ", len(tvals), " is ", totalLength, "\n"










