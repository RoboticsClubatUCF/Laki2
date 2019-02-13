from __future__ import division
import sys
import numpy as np
import scipy
import copy
import Polygon
import Polygon.Utils
import random
from scipy.interpolate import CubicSpline, interp1d
from scipy.optimize import minimize
from scipy import integrate
from scipy.misc import derivative
from scipy.signal import find_peaks
import matplotlib.pyplot as plt
from matplotlib.pyplot import Circle
from matplotlib.collections import PatchCollection

# Don't import like this for normal scripts. This ins only for testing
from cubicSplinesPath import *

def main():
    # Initialize Figure
    #fig, ax = plt.subplots()
    #plt.gca().set_xlim([0,  1300])
    #plt.gca().set_ylim([0, 1300])

    # Initialize 'given' waypoints
    wpxInit = [746.90, 1019.25, 390.52, 78.60, 204.4, 673.6]
    wpyInit = [1108.4, 1024.40, 155.47, 391.6, 612.7, 338.4]
    wpzInit = [100, 100, 100, 100, 100, 100, 100]

    # Comp Boundary converted to XY
    poly = [(609.9915651830946, 644.454456932276),
            (655.4769561099155, 1238.9138134970472),
            (899.4365305847842, 1268.1819761471047),
            (1240.810387266854, 1124.454562201312),
            (976.1887502964521, 788.4094397923109),
            (1029.310174576658, 466.5050901843899),
            (1188.824911231627, 309.8511306983688),
            (1002.0957697243854, 0.0036295812471155995),
            (421.55939798675325, 28.420887104681732),
            (0.03993415704199533, 366.0542881303085),
            (175.83977103766549, 764.1079686968526),
            (477.5319123645307, 629.0467535497892)]

    polyObj = Polygon.Polygon(poly)

    circles = makeRandomCircles(10, wpxInit, wpyInit, polyObj)

    ax = plt.gca()
    for circle in circles:
        circle = Circle(circle[0], circle[1], facecolor='r')
        ax.add_patch(circle)

    plt.plot(wpxInit, wpyInit, 'x', label = 'data', color = (0,0,0,1))

    # Makes parameter spacing to be constant
    t = [0]
    for i in xrange(len(wpxInit) - 1):
        dist = np.sqrt( (wpxInit[i] - wpxInit[i+1])**2 +
                        (wpyInit[i] - wpyInit[i+1])**2 +
                        (wpzInit[i] - wpzInit[i+1])**2)
        t.append(t[-1] + dist)


    #t = np.arange(len(wpxInit))
    print t

    xVals = []
    yVals = []
            
    for pt in poly:
        x, y = pt

        xVals.append(x)
        yVals.append(y)
    
    x, y = poly[0]
    xVals.append(x)
    yVals.append(y)
    plt.plot(xVals, yVals)

    csx = CubicSpline(t, wpxInit)
    csy = CubicSpline(t, wpyInit)

    # Plot parametric cubic splines
    s = t[-1] / 100000
    tSpace = np.arange(t[0], t[len(t)-1]+s, s)
    plt.plot(csx(tSpace), csy(tSpace))

    #plt.plot(csx(566.9), csy(566.9), 'o')
    #plt.plot(csx(2247), csy(2247), 'o')

    plt.show()
    
    wpxNew, wpyNew, tNew = fixCollisions(wpxInit, wpyInit, t, polyObj, circles, 2)

    print "sup"
    plotStuff(polyObj, None, circles, wpxNew, wpyNew, tNew)

    """

    #t = optimizeParameterSpacing(wpxInit, wpyInit, t, arcLength)[0]

    #print (t)
    csx = CubicSpline(t, wpxInit)
    csy = CubicSpline(t, wpyInit)

    # Plot parametric cubic splines
    s = t[-1] / 100000
    tSpace = np.arange(t[0], t[len(t)-1]+s, s)
    
    f, ((ax1), (ax2)) = plt.subplots(1, 2, sharey=False)
    ax1.plot(csx(tSpace), csy(tSpace))
    ax1.set_title("Path")
    ax1.plot(wpxInit, wpyInit, 'x', label = 'data', color = (0,0,0,1))

    # Curvature
    cur = []
    centerX = []
    centerY = []

    for val in tSpace:
        cur.append(curvature(val, csx, csy))

    ax2.plot(tSpace, cur)
    ax2.set_title("Curvature")

    peaks, _ = find_peaks(cur)

    print tSpace[peaks]

    for peak in peaks:
        key = False

        for pt in t:
            if (abs(tSpace[peak] - pt) < 1):
                key = True
                break

        if key:
            continue

        ax1.plot(csx(tSpace[peak]), csy(tSpace[peak]), 'o', color=(0,1,0,1))
        ax2.plot(tSpace[peak], cur[peak], 'o', color=(0,1,0,1))

    plt.show()

    """
    """
    # Concavity 

    con = []
    negcon = []

    for val in tSpace:
        con.append(concavity(val, csx, csy))
        negcon.append(-con[-1])


    ax2.plot(tSpace, con)
    ax2.set_title("Concavity")

    peaks, _ = find_peaks(con, height = 0.01)
    negPeaks, _ = find_peaks(negcon, height = 0.01)

    print tSpace[peaks], tSpace[negPeaks]

    for peak in peaks:
        ax1.plot(csx(tSpace[peak]), csy(tSpace[peak]), 'o', color=(0,1,0,1))
        ax2.plot(tSpace[peak], con[peak], 'o', color=(0,1,0,1))

    for peak in negPeaks:
        ax1.plot(csx(tSpace[peak]), csy(tSpace[peak]), 'o', color=(1,0,0,1))
        ax2.plot(tSpace[peak], con[peak], 'o', color=(1,0,0,1))


    plt.show()
    """

    """
    plt.plot(csx(tSpace), csy(tSpace))

    # Comp Boundary converted to XY
    poly = [(609.9915651830946, 644.454456932276),
            (655.4769561099155, 1238.9138134970472),
            (899.4365305847842, 1268.1819761471047),
            (1240.810387266854, 1124.454562201312),
            (976.1887502964521, 788.4094397923109),
            (1029.310174576658, 466.5050901843899),
            (1188.824911231627, 309.8511306983688),
            (1002.0957697243854, 0.0036295812471155995),
            (421.55939798675325, 28.420887104681732),
            (0.03993415704199533, 366.0542881303085),
            (175.83977103766549, 764.1079686968526),
            (477.5319123645307, 629.0467535497892)]
    """

    """
    polyObj = Polygon.Polygon(poly)

    xVals = []
    yVals = []
            
    for pt in poly:
        x, y = pt

        xVals.append(x)
        yVals.append(y)
    
    x, y = poly[0]
    xVals.append(x)
    yVals.append(y)
    plt.plot(xVals, yVals)
    
    circles = makeRandomCircles(30, wpxInit, wpyInit, polyObj)

    ax = plt.gca()
    for circle in circles:
        circle = Circle(circle[0], circle[1], facecolor='r')
        ax.add_patch(circle)

    plt.show()
    """

    """
    # Fix the intersections with the polygon
    wpxNew, wpyNew, t = fixCollisions(wpxInit, wpyInit, t.tolist(), polyObj, circles, 20)

    csx = CubicSpline(t, wpxNew)
    csy = CubicSpline(t, wpyNew)

    ax = plt.gca()
    for circle in circles:
        circle = Circle(circle[0], circle[1], facecolor='r')
        ax.add_patch(circle)

    # Plot parametric cubic splines
    s = 0.01
    tSpace = np.arange(t[0], t[len(t)-1] + s, s)
    plt.plot(csx(tSpace), csy(tSpace))

    plt.plot(wpxNew, wpyNew, 'x', label = 'data', color = (0,0,0,1))
    plt.plot(xVals, yVals)
    plt.show()
    """

    """
    # Starting Point will have some slope dependent on vehicle heading
    slope = (wpyInit[0] - wpyInit[1]) / (wpxInit[0] - wpxInit[1])
    slopeY = slope
    slopeX = 1.0

    if (wpyInit[1] < wpyInit[0]):
        slopeY *= -1
        slopeX *= -1

    # Same dy/dx for any c, used to clamp cubic in direction of vehicle heading
    c = 1
    slopeY *= c
    slopeX *= c

    # Create parametric cubic spline
    n = len(wpxInit)

    # Makes parameter spacing to be the euclidean distance between the waypoints
    t = [0]
    for i in xrange(len(wpxInit)):
        if (i == 0):
            continue

        t.append(t[-1] + np.linalg.norm([wpxInit[i] - wpxInit[i-1], 
                wpyInit[i] - wpyInit[i-1]]))

    # Makes parameter spacing to be constant
    t = np.arange(n)

    n = len(t)
    csx = CubicSpline(t, wpxInit, bc_type = ((1, slopeY), 'not-a-knot'))
    csy = CubicSpline(t, wpyInit, bc_type = ((1, slopeX), 'not-a-knot'))

    # Plot parametric cubic splines
    s = 0.01
    tSpace = np.arange(t[0], t[n-1] + s, s)
    plt.plot(csx(tSpace), csy(tSpace))

    circles = makeRandomCircles(30, wpxInit, wpyInit)

    for circle in circles:
        circle = Circle(circle[0], circle[1], facecolor='r')
        ax.add_patch(circle)

    # List of collision tuples
    collisions = []

    # Find the intersection points and pts of interest for each circle
    for circle in circles:
        collisions += cubicSplineCircleCollisions(csx, csy, t, circle) 

    print "\n", collisions

    # Plot the intersection points and points of interest
    intersectionPts = []
    ptsOfInterest = []
    
    for collision in collisions:
        intersections, ptOfInterest, cicle = collision
        
        intersectionPts.append(intersections[0])
        intersectionPts.append(intersections[1])
        
        ptsOfInterest.append(ptOfInterest)

    plt.plot(csx(intersectionPts), csy(intersectionPts), 'o', color = 'g')
    plt.plot(csx(ptsOfInterest), csy(ptsOfInterest), 'o', color = 'y')

    #t = optimizeParameterSpacing(wpxInit, wpyInit, t)

    collisions.sort(key=lambda x: x[1])

    for collision in collisions:
        x, y = csx(collision[1]), csy(collision[1])
    """


    # Create a line through a pt of interest and the circle it is inside of
    # Create a new waypoint along the line
    # Create a parameter q that determines the pos of the new waypoint on the line
    # When q is 0, the new waypoint is at the center of the circle
    # When q is + the waypoint moves away from the circle in one direction and when
    #   q is - the waypoint moves away from the circle in the other direction

    # Optomizer part is done, evaluation is TBD
    # Spacing for the splines is relative
    # [0, 1, 2, 3, 4] yields the same spline as [0, 2, 4, 6, 8]
    # Spacing for the splines is of the form [0, 1, a1, a2, a3, ...]
    # Where a1 is 1 + b1, a2 is a1 + b2, ...
    # (In this case 1 can be replaced with any arbitrary value

    # Variables to optimize: 
    #   all q's associated with intersection points
    #   b1, b2, b3, ...
    
    # Evaluation Function:
    #   Some function that takes the spline are returns the energy used to complete 
    #       the competition. Should calculate speed at each point based on curvature 
    #       and numerically solve by taking steps of step size speed * time_step
    
    # Additional concerns:
    #   New collisions will need to be dealt with accordingly
    #       Some collisions will be the result of the new, artificial waypoints 
    #       Some will be genuinely new problems
    #       Can determine which is which by seeing how close the 'new' collision is
    #           to the old collision






if __name__ == "__main__":
    main()


""" 
Testing Generation
Done    1) Map is 1.3 km x 0.8 km
Done    2) Randomly generate max 30 obstacles of radii from 
"""

"""
Path Generation
Done    1) Spline from position to goal
Done    2) Find points that intersect with circles
        3) Fix that
        4) Optimize for length/energy
Done        4a) Optimizer function
            4b) Evaluation Function
        5) Fix that
"""

"""
Display Map Steps:
        1) Rectangle for outer boundary
Done    2) Generate Random Circles
Done    3) Display circles
Done    4) Display Waypoints
        5) Display knot points
Done    6) Display splines
"""