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
from scipy import integrate, interpolate
from scipy.misc import derivative
import matplotlib.pyplot as plt
from matplotlib.pyplot import Circle
from matplotlib.collections import PatchCollection
import time

#----------------------------------------------------------------------------------#

# csx is the cubic spline of x as a function of t
# csy is the cubic spline of y as a function of t

# tVals is the list of parametric points serving as the knots for the cubic spline

# circle is of the form [(h, k), r] where (h, k) is the center and r is the radius
# returns a list of the values of t where the cubic intersects the circle

# segments in a list of the spline segments to iterate through. Must be continuous
#   Examples: 
#       To iterate through the first 5 segments of cubic spline:
#           segments = [0, 1, 2, 3, 4]
#       To iterate through the 3rd and 4th segment of a cubic spline:
#           segments = [2, 3]

# Returns a list of collision tuples. 
#   Each collision has the form (intersectionPts, ptOfInterest, circle)
#   Where intersectionPts are the two pts that intersect the circle. Only values
#       between the intersection points are inside the circle
#   ptOfInterest is the point between a par of intersection points that is closest 
#       the the center of the circle

def cubicSplineCircleCollisions(csx, csy, circle, segments=None):
    # Equation of a circle:
    #   r^2 = (x - h)^2 + (y - k)^2
    # x is defined in terms of t by csx
    # y is defined in terms of t by csy
    
    # Cubic spline object stores parameter values
    tVals = csx.x

    # collisions is the list to return
    collisions = []

    # Create a list object to store intersections with the circle
    intersections = []

    # Find the distance from the first endpoint to the center of the circle
    distToInitPt = np.linalg.norm([(csx(tVals[0]) - circle[0][0]), 
            (csy(tVals[0]) - circle[0][1])])

    # Find the distance from the last endpoint to the center of the circle
    distToFinPt = np.linalg.norm([(csx(tVals[-1]) - circle[0][0]), 
            (csy(tVals[-1]) - circle[0][1])])

    # Boolean value for logic
    resetCritPts = True

    # If nothing for segments is given, iterate throught the entirety of the list
    if (segments == None):
        segments = range(len(tVals) - 1)

    # Loop through all segments of the given cubic spline
    for i in segments:
        # tX is a list of the coefficients of the equation x = At^3 + Bt^2 + Ct + D
        # tY is a list of the coefficients of the equation y = at^3 + bt^2 + ct + d 
        tX = [csx.c[0][i], csx.c[1][i], csx.c[2][i], csx.c[3][i]]
        tY = [csy.c[0][i], csy.c[1][i], csy.c[2][i], csy.c[3][i]]

        # Combine constant terms from the circle and cubic
        # Inessense, modify the D and c terms to translate the center of the circle 
        #   to (0, 0)
        tX[3] -= circle[0][0]
        tY[3] -= circle[0][1]

        # Now, the equation can be of the form:
        #   r^2 = (tX)^2 + (tY)^2
        # Re-arrange to the following form and solve for t:
        #   0 = (tX)^2 + (tY)^2 - r^2

        tX2 = np.polymul(tX, tX)
        tY2 = np.polymul(tY, tY)
        coeff = np.polyadd(tX2, tY2)
        coeff = np.polysub(coeff, [circle[1]**2])

        # Finding the roots of the equation will find the para
        roots = np.roots(coeff)

        # Weed out the garbage points
        for root in roots:
            # Doesn't count if it is an imaginary root
            if not np.isreal(root):
                continue

            # The root must be between the start and end of this segment
            if (root >= 0 and root <= (tVals[i+1] - tVals[i])):
                intersections.append(root.real + tVals[i])

        # The end pts count as intersection points if they are inside the circle
        #   and we are analysing that particular segment
        if ((i == 0) and (distToInitPt <= circle[1])):
            # This avoids the edge case where the endpoint is on the circumference
            if (tVals[0] not in intersections):
                intersections.append(tVals[0])

        elif ((i == (len(tVals) - 2)) and (distToFinPt <= circle[1])): 
            # This avoids the edge case where the endpoint is on the circumference
            if (tVals[-1] not in intersections):
                intersections.append(tVals[-1])

        # Sort the intersection points to make manipulations easier
        intersections.sort()

        # If there are intersection points associated with this segment, find the
        #   the closest point and package it nicely into collisions
        if intersections:
            # The equation for dist from the center to the spline is of the form:
            #   p^2 = (tX)^2 + (tY)^2
            # Where p is the distance from the center to the spline

            # The derivative wrt t in its expanded form:
            #   2 * p * dp/dt = f(t)
            # Where f(t) is some function of t
            
            # We are interested in where p is at a minimum, so where dp/dt is 0
            # Since 2 & p are always positive, ignore them and use the eqn:
            #   dp/dt = d ((tX)^2 + (tY)^2) /dt
            # Use the derivative coefficients previously calculated for the
            #   polynomial describing the intersection points

            coeff = np.polyder(coeff)

            # The roots of that polynomial are the potential critical points where
            #   the distance to the center of the circle is at a local min or max
            roots = np.roots(coeff)

            # Only set critPts to the empty list if the points inside are no longer 
            #   needed
            if resetCritPts:
                critPts = []
            
            # Weed out the garbage points
            for root in roots:
                # Doesn't count if it is an imaginary root
                if not np.isreal(root):
                    continue

                # The root must be between the start and end of this segment
                if (root > 0 and root < (tVals[i+1] - tVals[i])):
                    critPts.append(root.real + tVals[i])

            critPts.append(tVals[i])
            critPts.append(tVals[i+1])

            # Sort the crit points to manipulations things easier
            critPts.sort()

            # Distance from the next knot to center of circle
            distToCenter = np.linalg.norm([(csx(tVals[i+1]) - circle[0][0]), 
                    (csy(tVals[i+1]) - circle[0][1])])

            # If the next knot is inside the circle, don't reset the critical points
            if (distToCenter <= circle[1]):
                resetCritPts = False

                # Unless this is the last segment, continue to the next segment
                if (i != len(tVals) - 2):
                    continue

            # whle there are still intersection points in the queue, process them
            while (len(intersections) > 1):
                dist = np.inf
                closestPt = -1

                # Note: only possible this easisly because critPts and intersections 
                #   are both respectively sorted

                # Find the closest critical point between the current pair of
                #   intersection points
                for crit in critPts:
                    # If this critical point is less than the first intersection 
                    #   point, continue to the next one
                    if (crit < intersections[0]):
                        continue

                    # If this critical point is greater than the second intersection
                    #   point, all future ones will also be greater, so the closest
                    #   point between the intersection points has already been found
                    if  (crit > intersections[1]):
                        break

                    # Otherwise, find the distance from this crit point to the 
                    #   center of the circle
                    newDist = np.linalg.norm([csx(crit) - circle[0][0], 
                            csy(crit) - circle[0][1]]) 

                    # Compare it to the previous best and update if necessary
                    if (newDist < dist):
                        dist = newDist
                        closestPt = crit                

                # Add this collision to the collisions return list
                collisions.append((
                        (intersections[0], intersections[1]),
                        closestPt,
                        circle))

                # Delete the leading pair of intersection points              
                del intersections[0]
                del intersections[0]

                # Since the statement before this while loop was not triggered, it
                #   is known that this is not an intermediate knot inside the circle
                # Because of that, there will be an even number of intersection
                #   points between the very start of the spline and the end of the
                #   current segement. Therefore, the intersections queue will be
                #   completely processsed and the critPts can be reset
                resetCritPts = True

    return collisions

#----------------------------------------------------------------------------------#

# csx is the cubic spline of x as a function of t
# csy is the cubic spline of y as a function of t

# tVals is the list of parametric points serving as the knots for the cubic spline

# segments in a list of the spline segments to iterate through. Must be continuous.
#   Examples: 
#       To iterate through the first 5 segments of cubic spline:
#           segments = [0, 1, 2, 3, 4]
#       To iterate through the 3rd and 4th segment of a cubic spline:
#           segments = [2, 3]

# Returns a list of collision tuples. 
#   Each collision has the form (intersectionPts, ptOfInterest, polySeg)
#   Where intersectionPts are the two values of the parameter that 
#       intersect the polygon. Only values between the intersection points are 
#       outside the polygon
#   ptOfInterest is the point half way between the pair of intersection points. It 
#       is usually the pt nearly the furthest outside of the polygon
#   polySeg is the line segment of intersection.
#       It is of the form ((x0, x1), (y0, y1))

def cubicSplinePolygonCollisions(csx, csy, polyObj, segments=None):
    # Helper Method:
    #   Returns True if given point is on line and false otherwise
    #   Endpoints are considered to be on the line
    #   segment is of the form: ((x1, x2), (y1, y2))
    #   pt is of the form (x, y)
    def isPtOnLine(segment, pt):
        # Define a machine precision value for floating point comparisons
        # Theoretically: sys.float_info.epsilon
        # In reality there are tolerance stack ups
        epsilon = 1 / 2**16
        
        # Returns True if the point is between the bounds of the segment and False
        #   otherwise
        def isInBounds(pt):
            # Because it is floating point numbers, we have to check if the number
            #   is inside very close to being inside the interval.
            #   numpy.isclose() does this check

            # If the intersection is within the x interval of the segment
            if (pt[0] >= xInterval[0]) and (pt[0] <= xInterval[1]):
                pass
            
            elif (np.isclose(pt[0], xInterval[0], 0, epsilon)
                    or np.isclose(pt[0], xInterval[1])):
                pass
            
            else:
                return False

            # If the intersection is within the y interval of the segment
            if (pt[1] >= yInterval[0]) and (pt[1] <= yInterval[1]):
                pass

            elif (np.isclose(pt[1], yInterval[0], 0, epsilon)
                    or np.isclose(pt[1], yInterval[1])):
                pass

            else:
                return False

            return True

        # segment is given in the form (x1, y1), (x2, y2)
        x1, x2 = segment[0]
        y1, y2 = segment[1]

        x3, y3 = pt

        # Find the x and y intervals of the line segments
        xInterval = [min(x1, x2), max(x1, x2)]
        yInterval = [min(y1, y2), max(y1, y2)]

        # Calculate the slopes of the line
        if (x1 - x2) == 0:
            slope = np.inf
        else:
            slope = (y1 - y2) / (x1 - x2)

        # special case when slope of line is infinite
        if slope == np.inf:
            # if the slope is infinite, but the x values aren't the same, it is not 
            #   on the line
            if not (np.isclose(x1, x3, 0, epsilon)):
                return False
            
            # if the point is above or below the line segment, it is not on the line
            if (y3 < min(y1, y2)) or (y3 > max(y1, y2)):
                return False

            # if they have the same x value, and the y value of the point is 
            #   sandwiched between the y values of the line segment, the point is on
            #   the line
            return True

        # When the slope isn't infinite, find the y intercept
        b = y1 - (slope * x1)

        # With the given line segment in y = mx + b form, plug in the x of the point
        #  If the resulting y is the same as the y of the pt, the pt is on the line
        #print ("(y3 - b): ", (y3 - b), "(slope * x3): ", (slope * x3))

        if (np.isclose((y3 - b), (slope * x3), 0, epsilon)):
            # Check that the point is within the bounds of the segment
            if isInBounds(pt):
                return True

            else:
                return False

        else:
            return False

    # Equation of a line:
    #   y = m * x + k
    # x is defined in terms of t by csx
    # y is defined in terms of t by csy

    tVals = csx.x

    # collisions is the list to return
    collisions = []

    intersections = []

    poly = Polygon.Utils.pointList(polyObj)

    if (segments == None):
        segments = range(len(tVals) - 1)

    # Loop through all segments of the given cubic spline
    for i in segments:
        # tX is a list of the coefficients of the equation x = At^3 + Bt^2 + Ct + D
        # tY is a list of the coefficients of the equation y = at^3 + bt^2 + ct + d 
        tX = [csx.c[0][i], csx.c[1][i], csx.c[2][i], csx.c[3][i]]
        tY = [csy.c[0][i], csy.c[1][i], csy.c[2][i], csy.c[3][i]]

        # Loop through all lines of the polygon
        for j in range(len(poly)):
            x = (poly[j-1][0], poly[j][0])
            y = (poly[j-1][1], poly[j][1])

            # True only if the respective endpoint is strictly outside the polygon
            isInitPtOutside = not (polyObj.isInside(csx(tVals[0]), csy(tVals[0])) 
                            or isPtOnLine((x, y), (csx(tVals[0]), csy(tVals[0]))))

            isFinPtOutside = not (polyObj.isInside(csx(tVals[-1]), csy(tVals[-1]))
                            or isPtOnLine((x, y), (csx(tVals[-1]), csy(tVals[-1]))))

            # Equation of a line
            # y = m*x + k

            # Calculate intersection points between line and cubic function
            #   y = m*x + k
            #   x = At^3 + Bt^2 + Ct + D
            #   y = at^3 + bt^2 + ct + d
            #   Therefore:
            #       0 = (a - m*A)t^3 + (b - m*B)t^2 + (c - m*C)t + (d - mD - k)

            if ((x[0] - x[1]) == 0):
                # Equation of a line is x = h bc slope is infinite
                # Intersection point is slightly easier
                # 0 = At^3 + Bt^2 + Ct + (D - h)
                coeff = copy.copy(tX)
                coeff[3] -= x[0]

            else:
                # Calculate the slope of the line
                m = (y[0] - y[1]) / (x[0] - x[1])
                
                # Calculate the y intercept of both line segments
                k = y[0] - (m * x[0])

                coeff = [(tY[0] - m*tX[0]),
                        (tY[1] - m*tX[1]), 
                        (tY[2] - m*tX[2]),
                        (tY[3] - m*tX[3] - k)]
            
            # Find the roots of the intersection equation
            roots = np.roots(coeff)

            # Weed out the garbage points
            for root in roots:
                # Doesn't count if it is an imaginary root
                if not np.isreal(root):
                    continue

                # The root must be between the start and end of this spline
                if (root >= 0 and root <= (tVals[i+1] - tVals[i])):
                    tPos = root.real + tVals[i]

                    loc = (csx(tPos).tolist(), csy(tPos).tolist())

                    # The root must be within line segment of the polygon
                    if (isPtOnLine((x, y), loc)):
                        intersections.append(root.real + tVals[i])

            # The end pts count as intersection points if they are outside the poly
            #   and we are analysing that particular segment
            if ((i == 0) and isInitPtOutside):
                # This avoids the edge case where the endpoint is on the poly
                if (tVals[0] not in intersections):
                    intersections.append(tVals[0])

            elif ((i == (len(tVals) - 2)) and isFinPtOutside): 
                # This avoids the edge case where the endpoint is on the poly
                if (tVals[-1] not in intersections):
                    intersections.append(tVals[-1])

            # Sort the intersection points to make manipulations easier
            intersections.sort()

            # From preliminary testing, previous to this line everything seems ok
            # Edge cases to test:
            #   Waypoint on the line
            #   polygon that completely swallows cubic spline
            #   polygon that intersects the same spline twice
            
            if (len(intersections) > 0):
                # While intersection points are still in the queue, process them
                while (len(intersections) > 1):
                    # Add this collision to the collisions return list
                    collisions.append((
                            (intersections[0], intersections[1]),
                            (intersections[0] + intersections[1])/2,
                            (x, y)))

                    # Delete the leading pair of intersection points              
                    del intersections[0]
                    del intersections[0]

    return collisions

#----------------------------------------------------------------------------------#

# wpx and wpy are the lists of waypoints
# tVals are the parameter values

# poly is a list of (x, y) vertice points of the outer polygon boundary
# circles is set of circles. Circles are of the form ((h, k), r) where (h, k) is the 
#   center and r is the radius

# i is the index of the spline segment that needs fixing

# allowedTime is the time allowed for the function to run in seconds
# NOTE: Function is written that it will continue past the allowed time if no
#   solution was found in the allowed time

# Through random guesses, this function finds individual intermediate waypoints that 
#   could be added to the given spline segment in order to stop it from having 
#   collisions.
# That is, it randomly guesses a point and inserts it into the spline segment. Then
#   it checks if this new spline segment intersects with the polygon. It does this
#   either until the max number of valid solutions is found, or the max number of
#   allowed guesses it hit and there is at least 1 solution

# TO DO: Don't use points that make the previously fixed splines collide again,
#   and/or need to do slight optimization of intermediate waypoints

# TO DO: Add heuristics to guesses

# TO DO: Create a polygon with which to guess points within
# p.sample(rng); 0.0 <= rng <= 1.0

def monteCarloSearch(wpx, wpy, tVals, newTval, polyObj, circles, i, allowedTime):
    # Start time of search
    startTime = time.time()

    # Make a copy of the inputs so as to not destory them
    wpxNew = copy.copy(wpx)
    wpyNew = copy.copy(wpy)
    
    tNew = copy.copy(tVals)
    tNew.insert(i+1, newTval)

    # TO DO:
    #   Modify the first new guess pt to be better.
    #   Ignore circle collisions. Consider no concavity sign change, etc

    seedPt = ((wpxNew[i] + wpxNew[i+1]) / 2, (wpyNew[i] + wpyNew[i+1]) / 2)

    wpxNew.insert(i+1, seedPt[0])
    wpyNew.insert(i+1, seedPt[1])

    # TO DO: make this dist not garbage. Take into account the dist to the 
    #   polygon, etc, the fact that it can move different distances in
    #   different directions, etc
    # Dist is the distance from the previous waypoint to the new 
    #   intermediate waypoint.

    dist = np.linalg.norm([(wpxNew[i], wpyNew[i]), (wpxNew[i+1], wpyNew[i+1])])

    # This keeps track of the number of points that create a valid solution
    solutions = []

    # Search for solutions until the allowed time is up and at least one solution
    #   has been found

    while (((time.time() - startTime) < allowedTime) or (len(solutions) < 1)):
        # TO DO: Add some better heuristic to this to make gueses more likely to
        #   stick and to make them better in general
        randPt = (seedPt[0] + dist/4 * np.random.normal(), 
                seedPt[1] + dist/4 * np.random.normal())

        wpxNew[i+1] = randPt[0]
        wpyNew[i+1] = randPt[1]

        csx = CubicSpline(tNew, wpxNew)
        csy = CubicSpline(tNew, wpyNew)

        # Check if there are any collisions with the polygon. If so, go back
        #   around to generate a new random point
        if (cubicSplinePolygonCollisions(csx, csy, polyObj, [i, i+1])):
            continue

        # Check if there are any collisions with the circle. If so, go back
        #   around to generate a new random point
        circCollision = False
        for circle in circles:
            # Check for circle collisions
            if (cubicSplineCircleCollisions(csx, csy, circle, [i, i+1])):
                circCollision = True
            
            # If at any point there is a circle collision, stop looking
            #   for more becasue this point is useless
            if circCollision:
                break

        # If there was a circle collision, go back to the start of the loop.
        #   Do not pass Go, do not collect $200
        if circCollision:
            continue

        solutions.append(randPt)
        print len(solutions)

    return solutions

#----------------------------------------------------------------------------------#

# wpx and wpy are the lists of waypoints
# tVals are the parameter values

# circles is set of circles. Circles are of the form ((h, k), r) where (h, k) is the 
#   center and r is the radius

# TO DO: 
#   Fix the spline segment by segment starting from the beginning
#   Feels like a single polygon intersection can be fixed with a single 
#       intermediate waypoint

def fixCollisions(wpx, wpy, tVals, polyObj, circles, allowedTime):
    csx = CubicSpline(tVals, wpx)
    csy = CubicSpline(tVals, wpy)

    # A copy of the original objects so we don't destory them
    wpyNew = copy.copy(wpy)
    wpxNew = copy.copy(wpx)
    tNew = copy.copy(tVals)

    # Count all the splines segment that have collisions
    numCollisions = 0

    # Iterate through all segments to count the number of troublesome segments
    for j in xrange(len(wpx)-1):
        # Find collisions between the polygon and the current spline segment
        if (len(cubicSplinePolygonCollisions(csx, csy, polyObj, [j])) > 0):
            numCollisions += 1
        else:
            for circle in circles:
                if (len(cubicSplineCircleCollisions(csx, csy, circle, [j])) > 0):
                    numCollisions += 1
                    break

    i = 0

    # Going to iterate through all segments of the spline
    while (i < (len(wpxNew) - 1)):
        print "i: ", i
        print "wpxNew: ", len(wpxNew)
        collisions = []

        # Find collisions between the polygon and the current spline segment
        collisions = cubicSplinePolygonCollisions(csx, csy, polyObj, [i])

        # If there is a collision with the polygon, fix it
        if len(collisions): 
            # TO DO:
            # Modify monte carlo such that it does not mess up previous segments
            # Modify monte carlo such that it generates more points depending on the
            #   a) number of collisions with the polygon and circles
            #   b) arc length of the segment
            solutions = monteCarloSearch(wpxNew, wpyNew, tNew, collisions[0][1], 
                    polyObj, circles, i, (allowedTime / (2*numCollisions)))
            print "Found ", len(solutions), " Solutions"

            # TO DO:
            #   Rn best point just returns the single 'best' point
            #   Modify this to return multiple pts and do a tree expansion on them 
            newPt = bestSolution(wpxNew, wpyNew, tNew, collisions[0][1], solutions, 
                    i+1, polyObj, circles)
            
            wpxNew.insert(i+1, newPt[0])
            wpyNew.insert(i+1, newPt[1])
            tNew.insert(i+1, collisions[0][1])

            csx = CubicSpline(tNew, wpxNew)
            csy = CubicSpline(tNew, wpyNew)

            plotStuff(polyObj, None, circles, wpxNew, wpyNew, tNew)
            continue

        for circle in circles:    
            collisions = cubicSplineCircleCollisions(csx, csy, circle, [i])
            if len(collisions):
                break

        # If there is a collision with the polygon, fix it
        if len(collisions): 
            # Put the intermediate point at the point of interest

            # TO DO:
            # Modify monte carlo such that it does not mess up previous segments
            # Modify monte carlo such that it generates more points depending on the
            #   a) number of collisions with the polygon and circles
            #   b) arc length of the segment
            solutions = monteCarloSearch(wpxNew, wpyNew, tNew, collisions[0][1], 
                    polyObj, circles, i, (allowedTime / (2*numCollisions)))
            print "Found ", len(solutions), " Solutions"

            # TO DO:
            #   Rn best point just returns the single 'best' point
            #   Modify this to return multiple pts and do a tree expansion on them 
            newPt = bestSolution(wpxNew, wpyNew, tNew, collisions[0][1], solutions, 
                    i+1, polyObj, circles)

            wpxNew.insert(i+1, newPt[0])
            wpyNew.insert(i+1, newPt[1])
            tNew.insert(i+1, collisions[0][1])

            csx = CubicSpline(tNew, wpxNew)
            csy = CubicSpline(tNew, wpyNew)

            plotStuff(polyObj, None, circles, wpxNew, wpyNew, tNew)

        i += 1

    return (wpxNew, wpyNew, tNew)

#----------------------------------------------------------------------------------#

def plotStuff(polyObj=None, poly=None, circles =None, wpx=None, wpy=None, tVals=None, 
                csx=None, csy=None):
    
    if (poly == None and polyObj != None): 
        poly = Polygon.Utils.pointList(polyObj)

    if (poly != None):
        # Plot the polygon
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
    
    if (circles != None):
        # Plot the circles
        ax = plt.gca()
        for circle in circles:
            circle = Circle(circle[0], circle[1], facecolor='r')
            ax.add_patch(circle)

    if (wpx != None and wpy != None):
        plt.plot(wpx, wpy, 'x', label = 'data', color = (0,0,0,1))

        if (tVals != None):
            if (csx == None):
                csx = CubicSpline(tVals, wpx)

            if (csy == None):
                csy = CubicSpline(tVals, wpy)

    if (tVals == None and csx != None):
        tVals = csx.x

    if (csx != None and csy != None and tVals != None):
        # Plot the path
        s = 0.01
        tSpace = np.arange(tVals[0], tVals[-1] + s, s)
        plt.plot(csx(tSpace), csy(tSpace))

    plt.show()
    return

#----------------------------------------------------------------------------------#

def bestSolution(wpx, wpy, tVals, newTval, points, index, polyObj, circles):
    # Calculate the length and curvature of the path without modification
    csx = CubicSpline(tVals, wpx)
    csy = CubicSpline(tVals, wpy)
    baseLen = length = arcLength(csx, csy, csz=None)
    baseCurv = integrate.quad(lambda x: curvature(x, csx, csy), tVals[0], tVals[-1], 
            limit = 75)[0]

    scores = []

    lengths = []
    curvs = []

    for point in points:
        wpxNew = copy.copy(wpx)
        wpyNew = copy.copy(wpy)
        tNew = copy.copy(tVals)

        wpxNew.insert(index, point[0])
        wpyNew.insert(index, point[1])
        tNew.insert(index, newTval)

        csx = CubicSpline(tNew, wpxNew)
        csy = CubicSpline(tNew, wpyNew)

        length = arcLength(csx, csy, csz=None)

        curv = integrate.quad(lambda x: curvature(x, csx, csy), 
                           tNew[0], tNew[-1], limit = 75)[0]
        
        lengths.append((baseLen - length) / baseLen)
        curvs.append((baseCurv - curv) / baseCurv)

    for i in xrange(len(points)):
        length = lengths[i]
        curv = curvs[i]
        point = points[i]

        # Score here is set with meta-parameters
        # Choosen by operator based on desired output
        # By setting alpha it changes how much weight is put on length vs curvature

        # TO DO: Modify this so it calculates the energy of the path
        # 

        alpha = 0.3
        print "length: ", np.round(length, 4), "\talpha * curv: ", np.round(alpha*curv, 4)
        score = length + alpha * np.sign(curv) * np.sqrt(abs(curv))

        scores.append((score, point))

    scores.sort()

    for i in xrange(len(scores)):
        wpxNew = copy.copy(wpx)
        wpyNew = copy.copy(wpy)
        wpxNew.insert(index, scores[i][1][0])
        wpyNew.insert(index, scores[i][1][1])

        print scores[i][0]
        plotStuff(polyObj, None, circles, wpxNew, wpyNew, tNew)

    return scores[0][1]

#----------------------------------------------------------------------------------#

# Given 3 lists, a list for each of the x, and y coordinates of waypoints, as well
#   as an init spacing for the parameter. Initial spacing must be stricly increasing
# Lists need to be of the same length
# Pass a costFunction that evaluates the cost of the spline

# If successful, returns (t_opt, True) where t_opt is the optimal spacing
# If unsuccessful, returns (t, False) where t is the original spacing
def optimizeParameterSpacing(wpx, wpy, tVals, costFunc):
    def evalSpacing(x, tVals):
        # The first two parameters are fixed
        # Any scalar multiple of the spacing results in the same spline
        spacing = [tVals[0], tVals[1]]

        # Add the other (n-2) terms to t
        for param in x:
            # x contains the difference between the current and previous index
            # Exponential is used to ensure the value is strictly positive
            # This ensures that t is strictly increasing
            spacing.append(spacing[-1] + np.exp(param))

        csx = CubicSpline(spacing, wpx)
        csy = CubicSpline(spacing, wpy)

        # TO DO: make this use the actual z spline
        # Just for testing, keeps z at same altitude the whole time
        wpz = np.array(len(wpx))
        wpz.fill(200)
        csz = CubicSpline(spacing, wpy)

        # Return some cost of the spline
        length = costFunc(csx, csy, csz, spacing)
        print length
        return length

    #--------------------------------------------------------------------------#

    x0 = []
    for i in range(len(tVals)):
        if (i == 0 or i == 1):
            continue

        # Difference between the current     
        diff = tVals[i] - tVals[i-1]
        x0.append(np.log(diff))

    # x is now an array representing the natural log of the difference between 
    #   non-fixed values of t
    x0 = np.array(x0)

    # To aquire the respective spacing, simply exponentiate an element of x and add 
    #   it to the previous element in the spacing
    # This allows for the elements of x to take on any real value while ensuring
    #   that the respective spacing is strictly increasing
    opt_obj = minimize(evalSpacing, x0, tVals, method = 'Nelder-Mead', tol = 1e-8, 
                    options = {'maxiter':5000})

    # Determine if the optimization was successful (it should be in all cases)
    if (opt_obj.success):
        x_opt = opt_obj.x

        # The first two parameters are fixed
        # Any scalar multiple of the spacing results in the same spline
        t_opt = [tVals[0], tVals[1]]

        # Add the other (n-2) terms to t
        for param in x_opt:
            # x contains the difference between the current and previous index
            # This ensures that t is monotomically increasing
            t_opt.append(t_opt[-1] + np.exp(param))

        return (t_opt, True)

    # If unsucessful, deal with it. Life isn't fair
    else:
        return (tVals, False)

#----------------------------------------------------------------------------------#

# csx is the cubic spline of x as a function of t
# csy is the cubic spline of y as a function of t
# csz is the cubic spline of z as a function of t (optional)

# tVals is the list of parametric points serving as the knots for the cubic spline

# Returns the arc length of the parametric line between the bounds of the parameter
# Supports 2D and 3D splines

# Written by Andrew Schroeder on 9/13/2018

def arcLength(csx, csy, csz=None):
    tVals = csx.x
    totalLength = 0

    # If there was no z spline given, make it sit flat on the x-y plane
    # The arc length will just be from the x and y splines
    if (csz == None):
        wpz = copy.copy(csx.x)
        wpz.fill(0)
        csz = CubicSpline(tVals, wpz)

    # Loop thru all splines in the given object
    for i in range(len(tVals) - 1):
        # tX is a list of the coefficients of the equation x = At^3 + Bt^2 + Ct + D
        # tY is a list of the coefficients of the equation y = at^3 + bt^2 + ct + d
        # tZ is a list of the coefficients of the equation z = aat^3 + bbt^2 + cct + dd
        tX = [csx.c[0][i], csx.c[1][i], csx.c[2][i], csx.c[3][i]]
        tY = [csy.c[0][i], csy.c[1][i], csy.c[2][i], csy.c[3][i]]
        tZ = [csz.c[0][i], csz.c[1][i], csz.c[2][i], csz.c[3][i]]
        
        # define the parametric functions fx_dt(t), fy_dt(t), and fz_dt(t)
        def fx_dt(t):
            return 3*tX[0]*t**2 + 2*tX[1]*t + tX[2]
        
        def fy_dt(t): 
            return 3*tY[0]*t**2 + 2*tY[1]*t + tY[2]

        def fz_dt(t): 
            return 3*tZ[0]*t**2 + 2*tZ[1]*t + tZ[2]

        # Integrate the square root of the sum of dX^2, dY^2, dZ^2 
        #   with respect to t from t = i to t = i + 1
        length = integrate.quad(
                        lambda p: np.sqrt(fx_dt(p)**2 + fy_dt(p)**2 + fz_dt(p)**2), 
                        tVals[i], tVals[i + 1], limit = 75)[0]

        # add the segment length to the total path length
        totalLength += length

    return totalLength

#----------------------------------------------------------------------------------#

# csx is the cubic spline of x as a function of t
# csy is the cubic spline of y as a function of t

# tVals is the list of parametric points serving as the knots for the cubic spline

# t is a location on the spline

# Returns the concavity of the spline at a given location

# Written by Alex Damis on 01/09/19

def concavity(t, csx, csy):
        tVals = csx.x
        i = 0

        for k in range(len(tVals)):
            if tVals[k] > t:
                i = k - 1
                break

        tX = [csx.c[0][i], csx.c[1][i], csx.c[2][i], csx.c[3][i]]
        tY = [csy.c[0][i], csy.c[1][i], csy.c[2][i], csy.c[3][i]]
        
        # First Derivative of tX and tY
        dydt = np.polyder(tY)
        dxdt = np.polyder(tX)

        # Second Derivative of tX and tY
        d2ydt2 = np.polyder(dydt)
        d2xdt2 = np.polyder(dxdt)

        # Quotient rule
        numer = np.polysub(np.polymul(d2ydt2, dxdt), np.polymul(d2xdt2, dydt))
        denom = np.polymul(dxdt, dxdt)

        q, r = np.polydiv(numer, denom)

        q_val = np.polyval(q, t)
        r_val = np.polyval(r, t) / np.polyval(denom, t)

        # Find all the spikes here. Not done, this doesn't work yet
        roots = np.roots(denom)
        for root in roots:
            # If the complex part of the root is significant, ignore that root
            # Significant is defined as 0.001% of the value of the root
            if abs(np.imag(root)) > abs(root/1e5):
                continue

            # Otherwise assume the imaginary portion is within numerical tolerance
            else:
                root = np.real(root)

            # If the root is close to t, it is a pole
            # Close is defined as 0.1% of the value of t
            if (abs(root - t) < abs(t/1e3)):
                print "Infinity: ", np.real(root)

        return q_val + r_val

        """

        # Quotient rule
        numer_eq = np.polysub(np.polymul(d2ydt2, dxdt), np.polymul(d2xdt2, dydt))
        numer_val = np.polyval(numer_eq, t)

        denom_eq = np.polymul(dxdt, dxdt)
        denom_val = np.polyval(denom_eq, t)

        return numer_val / denom_val
        """

#----------------------------------------------------------------------------------#

def center_of_curvature(t, csx, csy):
    tVals = csx.x
    i = 0

    for k in range(len(tVals)):
        if tVals[k] > t:
            i = k - 1
            break

    # Curvature = abs(T(t) / ds)
    #   where T is the unit tangent vector and ds is the arc length
    # This can be re-written as
    #   Curvature = abs(r'(t) x r"(t)) / (abs(r'(t))^3)
    #   where r is the function

    rX = [csx.c[0][i], csx.c[1][i], csx.c[2][i], csx.c[3][i]]
    rY = [csy.c[0][i], csy.c[1][i], csy.c[2][i], csy.c[3][i]]

    # First derivative of rX, rY, and rZ
    dxdt = np.polyder(rX)
    dydt = np.polyder(rY)

    # Second derivative of rX, rY, and rZ
    d2xdt2 = np.polyder(dxdt)
    d2ydt2 = np.polyder(dydt)

    dxdt = np.polyval(dxdt, t)
    dydt = np.polyval(dydt, t)
    d2xdt2 = np.polyval(d2xdt2, t)
    d2ydt2 = np.polyval(d2ydt2, t)

    Cx = csx(t) - dydt * (dxdt**2 + dydt**2) / (dxdt*d2ydt2 - d2xdt2*dydt)
    Cy = csy(t) + dxdt * (dxdt**2 + dydt**2) / (dxdt*d2ydt2 - d2xdt2*dydt)

    return (Cx, Cy)

#----------------------------------------------------------------------------------#

def curvature(t, csx, csy, csz=None):
    tVals = csx.x
    i = 0

    for k in range(len(tVals)):
        if tVals[k] > t:
            i = k - 1
            break

    """
    # If there was no z spline given, make it sit flat on the x-y plane
    # The arc length will just be from the x and y splines
    if (csz == None):
        wpz = copy.copy(csx.x)
        wpz.fill(0)
        csz = CubicSpline(tVals, wpz)
    """

    # Curvature = abs(T(t) / ds)
    #   where T is the unit tangent vector and ds is the arc length
    # This can be re-written as
    #   Curvature = abs(r'(t) x r"(t)) / (abs(r'(t))^3)
    #   where r is the function

    rX = [csx.c[0][i], csx.c[1][i], csx.c[2][i], csx.c[3][i]]
    rY = [csy.c[0][i], csy.c[1][i], csy.c[2][i], csy.c[3][i]]
    #rZ = [csz.c[0][i], csz.c[1][i], csz.c[2][i], csz.c[3][i]]

    # First Derivative of rX, rY, and rZ
    dxdt = np.polyder(rX)
    dydt = np.polyder(rY)
    #dzdt = np.polyder(rZ)

    drdt = [np.polyval(dxdt, t), np.polyval(dydt, t)] #, np.polyval(dzdt, t)]

    d2xdt2 = np.polyder(dxdt)
    d2ydt2 = np.polyder(dydt)
    #d2zdt2 = np.polyder(dzdt)

    d2rdt2 = [np.polyval(d2xdt2, t), np.polyval(d2ydt2, t)] #, np.polyval(d2zdt2, t)]

    numer = np.linalg.norm(np.cross(d2rdt2, drdt))
    denom = abs(np.linalg.norm(drdt))**3

    #To DO:
    #   the rest
    return numer / denom

#----------------------------------------------------------------------------------#

# Helper function for main. Returns a list of circle tuples
# Each circle tuple is of the form ((h, k), r) where (h, k) is the center and r is
#   the radius

def makeRandomCircles(numCircles, wpx, wpy, polyObj):
    import random

    poly = Polygon.Utils.pointList(polyObj)

    # Put given waypoints into a list of (x, y) points
    pts = []
    for i in range(len(wpx)):
        pt = (wpx[i], wpy[i])
        pts.append(pt)

    # Generate Random Obstacles
    count = 0
    circles = set()

    # Create n random, non-overlapping circles
    while (count < numCircles):
        # Random center and radius
        h = random.uniform(0.0, 1300.0)
        k = random.uniform(0.0, 1300.0)
        r = random.uniform(30*12*0.0254, 300*12*0.0254)

        # Determine if the circle contains a waypoint
        key = False
        for pt in pts:
            if (np.linalg.norm((pt[0] - h, pt[1] - k)) < (r * 1.1)):
                key = True
                break

            else:
                for circle in circles:
                    distBtwnCircles = np.linalg.norm([circle[0][0] - h, 
                                                    circle[0][1] - k])
                    
                    if (distBtwnCircles < (circle[1] + r)* 1.1):
                        key = True
                        break

        # If any of the waypoints are inside the circle, don't use this circle
        if key:
            continue

        # If the center of the circle is outside the polygon, don't use this circle
        if (not polyObj.isInside(h, k)):
            continue

        # Add circle to plot and list of circles
        circles.add(((h, k), r))
        count += 1

    return circles