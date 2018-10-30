from __future__ import division
import sys
import numpy as np
import scipy
import copy
import Polygon
import random
from scipy.interpolate import CubicSpline, interp1d
from scipy.optimize import minimize
from scipy import integrate
from scipy.misc import derivative
import matplotlib.pyplot as plt
from matplotlib.pyplot import Circle
from matplotlib.collections import PatchCollection

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

def cubicSplineCircleCollisions(csx, csy, tVals, circle, segments=None):
    # Equation of a circle:
    #   r^2 = (x - h)^2 + (y - k)^2
    # x is defined in terms of t by csx
    # y is defined in terms of t by csy

    # collisions is the list to return
    collisions = []

    intersections = []

    distToInitPt = np.linalg.norm([(csx(tVals[0]) - circle[0][0]), 
            (csy(tVals[0]) - circle[0][1])])

    distToFinPt = np.linalg.norm([(csx(tVals[-1]) - circle[0][0]), 
            (csy(tVals[-1]) - circle[0][1])])

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
        # This can be expanded to:
        #   0 = (A^2 + a^2)t^6 + 
        #       (2AB + 2ab)t^5 + 
        #       (2AC + B^2 + 2ac + b^2)t^4 +
        #       (2AD + 2BC + 2ad + abc)t^3 + 
        #       (2BD + C^2 + 2bd + c^2)t^2 +
        #       (2CD + 2cd)t + 
        #       (D^2 + d^2 - r^2)

        coeff = [(tX[0]**2 + tY[0]**2),
                (2 * tX[0] * tX[1] + 2 * tY[0] * tY[1]),
                (2 * tX[0] * tX[2] + tX[1]**2 + 2 * tY[0] * tY[2] + tY[1]**2),
                (2 * tX[0] * tX[3] + 2 * tX[1] * tX[2] + 
                    2 * tY[0] * tY[3] + 2 * tY[1] * tY[2]),
                (2 * tX[1] * tX[3] + tX[2]**2 + 2 * tY[1] * tY[3] + tY[2]**2), 
                (2 * tX[2] * tX[3] + 2 * tY[2] * tY[3]),
                (tX[3]**2 + tY[3]**2 - circle[1]**2)]

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
            # Use the power rule on the coefficients previously calculated for the
            #   polynomial describing the intersection points

            coeff = [6 * coeff[0],
                    5 * coeff[1],
                    4 * coeff[2],
                    3 * coeff[3],
                    2 * coeff[4], 
                    1 * coeff[5]]

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
#   Where intersectionPts are the two pts that intersect the polygon. Only values
#       between the intersection points are outside the polygon
#   ptOfInterest is the point between a par of intersection points that is farthest 
#       outside the polygon
#   polySeg is the line segment of intersection.
#       It is of the form ((x0, x1), (y0, y1))

# TO DO: Pass this a Polygon object from the library, or both the list and Polygon
#   object. Should be faster since it doesn't have to make the object every time

def cubicSplinePolygonCollisions(csx, csy, tVals, poly, segments=None):
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
            #print ("Is on line")

            # Check that the point is within the bounds of the segment
            if isInBounds(pt):
                #print ("Is within Bounds")
                return True

            else:
                #print ("Is NOT within Bounds")
                return False

        else:
            #print ("Is NOT on line")
            return False

    # Equation of a line:
    #   y = m * x + k
    # x is defined in terms of t by csx
    # y is defined in terms of t by csy

    # collisions is the list to return
    collisions = []

    intersections = []

    polyObj = Polygon.Polygon(poly)

    resetCritPts = True

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

            # Past this line is to find the pointOfInterest associated with each
            #   pair of intersection points
            # Nothing has been tested yet
            
            if (len(intersections) > 0):
                # Only set critPts to the empty list if the points inside are no 
                #   longer needed
                if resetCritPts:
                    critPts = []

                # Equation of a line is:
                #   y = m*x + k
                
                # Alternate equation of a line:
                #   E * x + F * y + G = 0
                
                # Distance from point (x0, y0) to line:
                #   dist = abs(E*x0 + F*y0 + G) / sqrt(E^2 + F^2)
                
                # Alternative Form:
                #   dist = sqrt((E*x0 + F*y0 + G)^2) / sqrt(E^2 + F^2)
                #   dist^2 = (E*x0 + F*y0 + G)^2) / (E^2 + F^2)

                # From the first eqution:
                #   m * x + -1 * y + k = 0
                
                # Any point (x0, y0) of the cubic spline is a function of t:
                #   x = At^3 + Bt^2 + Ct + D
                #   y = at^3 + bt^2 + ct + d

                # Therefore:
                #   dist^2 = [(m * (At^3 + Bt^2 + Ct + D) + 
                #           -1 * (at^3 + bt^2 + ct + d) + k]^2 / [m^2 + 1]

                # Critical Points where dist is at a maximum or minimum:
                #   2 * dist * d(dist)/dt = 
                #       2 * [m * (3*A*t^2 + 2*B*t + C) - (3*t^2 + 2*b*t + c)] * 
                #       [(m * (At^3 + Bt^2 + Ct + D) - (at^3 + bt^2 + ct + d) + k] * 
                #       1 / [m^2 + 1]

                #   Solve for when d(dist)/dt = 0:
                #   2, dist, and [m^2 +1] always non-0 between intersection interval
                #       0 = [m * (3*A*t^2 + 2*B*t + C) - (3*t^2 + 2*b*t + c)]
                #       or
                #       0 = (m * (At^3 + Bt^2 + Ct + D) - (at^3 + bt^2 + ct + d) + k
                #   The second term is just the intersection points, which are 
                #       garaunteed to be local minima (dist = 0)
                
                # Algebraic Manipulation:
                #       0 = (3*m*A - 3*a) * t^2 + (2*m*B - 2*b)* t + (m*C - c)

                if ((x[0] - x[1]) == 0):
                    coeff = [3*tX[0],
                            2*tX[1],
                            tX[2]]

                else:
                    coeff = [3*m*tX[0] - 3*tY[0],
                            2*m*tX[1] - 2*tY[1],
                            m*tX[2] - tY[2]]

                # Find the roots of the critical points equation
                roots = np.roots(coeff)

                # Weed out the garbage points
                for root in roots:
                    # Doesn't count if it is an imaginary root
                    if not np.isreal(root):
                        continue

                    # The root must be between the start and end of this segment
                    if (root >= 0 and root <= (tVals[i+1] - tVals[i])):
                        critPts.append(root.real + tVals[i])

                isOutside = (isPtOnLine((x, y), (tVals[i], tVals[i])) or 
                                    polyObj.isInside(tVals[i], tVals[i]))

                if isOutside:
                    critPts.append(tVals[i])

                isOutside = (isPtOnLine((x, y), (tVals[i+1], tVals[i+1])) or 
                                    polyObj.isInside(tVals[i+1], tVals[i+1]))

                if isOutside:
                    critPts.append(tVals[i+1])

                # Sort the crit points to manipulations things easier
                critPts.sort()

                # Boolean for if the next knot is strictly outside the polygon
                isNextKnotOutside = (isPtOnLine((x, y), (tVals[i+1], tVals[i+1])) or 
                                    polyObj.isInside(tVals[i+1], tVals[i+1]))

                # If next knot is outside the polygon, don't reset the crit points
                if (isNextKnotOutside):
                    resetCritPts = False

                    # Unless this is the last segment, continue to the next segment
                    if (i != len(tVals) - 2):
                        continue

                # While intersection points are still in the queue, process them
                while (len(intersections) > 1):
                    dist = -np.inf
                    closestPt = -1

                    # Note: only possible this easisly because critPts and 
                    #   intersections are both respectively sorted

                    # Find the closest critical point between the current pair of
                    #   intersection points
                    for crit in critPts:
                        # If this critical point is less than the first intersection 
                        #   point, continue to the next one
                        if (crit <= intersections[0]):
                            continue

                        # If this critical point is greater than the second 
                        #   intersection point, all future ones will also be 
                        #   greater, so the closest point between the intersection 
                        #   points has already been found
                        if  (crit >= intersections[1]):
                            break

                        # Distance from point (x0, y0) to line:
                        #   dist = abs(m*x0 + -1*y0 + k) / sqrt(m^2 + 1)

                        # Otherwise, find the distance from this crit point to the 
                        #   polygon
                        newDist = abs(m*csx(crit) - csy(crit) + k) 
                        newDist /= np.sqrt(m**2 + 1)

                        # Compare it to the previous best and update if necessary
                        if (newDist > dist):
                            dist = newDist
                            closestPt = crit                

                    # Add this collision to the collisions return list
                    collisions.append((
                            (intersections[0], intersections[1]),
                            closestPt,
                            (x, y)))

                    # Delete the leading pair of intersection points              
                    del intersections[0]
                    del intersections[0]

                    # Since the statement before this while loop was not triggered, 
                    #   it is known that this is not an intermediate knot outside
                    #   the polygon.
                    # Because of that, there will be an even number of intersection
                    #   points between the very start of the spline and the end of 
                    #   the current segement. Therefore, the intersections queue 
                    #   will be completely processsed and the critPts can be reset
                    resetCritPts = True

    return collisions

#----------------------------------------------------------------------------------#

# wpx and wpy are the lists of waypoints
# tVals are the parameter values

# poly is a list of (x, y) vertice points of the outer polygon boundary
# circles is set of circles. Circles are of the form ((h, k), r) where (h, k) is the 
#   center and r is the radius

# i is the index of the spline segment that needs fixing

# Through random guesses, this function finds individual intermediate waypoints that 
#   could be added to the given spline segment in order to stop it from having 
#   collisions.
# That is, it randomly guesses a point and inserts it into the spline segment. Then
#   it checks if this new spline segment intersects with the polygon. It does this
#   either until the max number of valid solutions is found, or the max number of
#   allowed guesses it hit and there is at least 1 solution

# TO DO: Create a potential fields solution as a heuristic (in another function) 
#   Potential fields -> stream function.
#   Find streamline(s) that get to the goal
#   Randomly sample points along the streamline(s) 

# TO DO: Don't use points that make the previously fixed splines collide again,
#   and/or need to do slight optimization of intermediate waypoints

def monteCarloSearch(wpx, wpy, tVals, poly, circles, i, maxGuesses, maxSolutions):
    # Make a copy of the inputs so as to not destory them
    wpxNew = copy.copy(wpx)
    wpyNew = copy.copy(wpy)
    tNew = copy.copy(tVals)

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

    # This keeps track of the number of guesses taken
    guessCounter = 0

    # This keeps track of the number of points that create a valid solution
    solutions = []

    # Search for solutions until the max number of solutions is found
    while (len(solutions) < maxSolutions):
        # If the number of guesses exceeds the maxGuesses and there is at least one
        #   solution, stop searching for solutions
        if (guessCounter > maxGuesses and len(solutions) > 0):
            break

        # TO DO: Add some better heuristic to this to make gueses more likely to
        #   stick and to make them better in general
        randPt = (seedPt[0] + dist/4 * np.random.normal(), 
                seedPt[1] + dist/4 * np.random.normal())

        guessCounter += 1

        wpxNew[i+1] = randPt[0]
        wpyNew[i+1] = randPt[1]

        csx = CubicSpline(tNew, wpxNew)
        csy = CubicSpline(tNew, wpyNew)

        # Check if there are any collisions with the polygon. If so, go back
        #   around to generate a new random point
        if (cubicSplinePolygonCollisions(csx, csy, tNew, poly, [i, i+1])):
            continue

        # Check if there are any collisions with the circle. If so, go back
        #   around to generate a new random point
        circCollision = False
        for circle in circles:
            # Check for circle collisions
            if (cubicSplineCircleCollisions(csx, csy, tNew, circle, [i, i+1])):
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
        print (len(solutions))

    return solutions

#----------------------------------------------------------------------------------#

# wpx and wpy are the lists of waypoints
# tVals are the parameter values

# poly is a list of (x, y) vertice points of the outer polygon boundary
# circles is set of circles. Circles are of the form ((h, k), r) where (h, k) is the 
#   center and r is the radius

# TO DO: 
#   Fix the spline segment by segment starting from the beginning
#   Feels like a single polygon intersection can be fixed with a single 
#       intermediate waypoint

# TO DO:
#   (In another function) To fix the circle obstacles, may need one waypoint per 
#       circle collision. Try putting the waypoint on both sides of the circle to 
#       see whether shifting cw or ccw is better

# Super TO DO:
#   This is going probably going to have to become some search algorithm
#   'Legal Action Generator' function almost complete 
#       (monteCarloSearch finds points that work, needs to make sure it doesn't mess 
#       up downstream segements)
#   'State Generator' function is complete (scipy's cubic spline generator)
#   'Cost Function' not started. For now probs minimize sum of square of curvature
#   'A*' should be able to be copied from CS188 files
#   'Actions' can be thought of as the additional points between real waypoints

def fixPolygonIntersections(wpx, wpy, tVals, poly, circles):
    csx = CubicSpline(tVals, wpx)
    csy = CubicSpline(tVals, wpy)

    # A copy of the original objects so we don't destory them
    wpyNew = copy.copy(wpy)
    wpxNew = copy.copy(wpx)
    tNew = copy.copy(tVals)

    #polyObj = Polygon.Polygon(poly)

    # Start with the first spline segment
    i = 0

    # TO DO: Probably want it to loop back around to check real quick that a 
    #   previous spline doesn't get screwed up
    # Going to iterate through all segments of the spline
    while (i < (len(wpxNew) - 1)):
        # Find collisions between the polygon and the current spline segment
        collisions = cubicSplinePolygonCollisions(csx, csy, tNew, poly, [i])

        # If there is a collision
        if len(collisions): 
            # Put the intermediate point at the point of interest
            tNew.insert(i+1, collisions[0][1])

            solutions = monteCarloSearch(wpxNew, wpyNew, tNew, poly, circles, i, 3000, 20)
            print ("Found")

            # TO DO: Evaulate all points in solutions to pick the best
            #   Do that here instead of just taking the first solution
            bestPt = solutions[0]
            wpxNew.insert(i+1, bestPt[0])
            wpyNew.insert(i+1, bestPt[1])

            csx = CubicSpline(tNew, wpxNew)
            csy = CubicSpline(tNew, wpyNew)
        
            # TO DO: will eventually need to remove the plotting
            # Plot parametric cubic splines

            plt.plot(wpxNew, wpyNew, 'x', label = 'data', color = (0,0,0,1))

            # Plot the path
            s = 0.01
            tSpace = np.arange(tNew[0], tNew[len(tNew)-1] + s, s)
            plt.plot(csx(tSpace), csy(tSpace))
            
            # Plot the circles
            ax = plt.gca()
            for circle in circles:
                circle = Circle(circle[0], circle[1], facecolor='r')
                ax.add_patch(circle)

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

            plt.show()


        # Move on to the next spline segment
        i += 1

    return (wpxNew, wpyNew, tNew)

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
        print (length)
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

def arcLength(tVals, csx, csy, csz=None):
    totalLength = 0

    # If there was no z spline given, make it sit flat on the x-y plane
    # The arc length will just be from the x and y splines
    if (csz == None):
        wpz = csx.x
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
        length = integrate.quadrature(
                lambda p: np.sqrt(fx_dt(p)**2 + fy_dt(p)**2 + fz_dt(p)**2), 
                tVals[i], tVals[i + 1])
        
        length = length[0]

        # add the segment length to the total path length
        totalLength += length

    return totalLength

#----------------------------------------------------------------------------------#

# Helper function for main. Returns a list of circle tuples
# Each circle tuple is of the form ((h, k), r) where (h, k) is the center and r is
#   the radius

def makeRandomCircles(numCircles, wpx, wpy, poly):
    import random

    polyObj = Polygon.Polygon(poly)

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