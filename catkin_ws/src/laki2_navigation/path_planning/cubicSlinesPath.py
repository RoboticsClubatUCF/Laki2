from __future__ import division
import sys
import numpy as np
import scipy
from scipy.interpolate import CubicSpline, interp1d
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from matplotlib.pyplot import Circle
from matplotlib.collections import PatchCollection

#----------------------------------------------------------------------------------#

# csx is the cubic spline of x as a function of t
# csy is the cubic spline of y as a function of t

# tVals is the list of parametric points serving as the knots for the cubic spline

# circle is of the form [(h, k), r] where (h, k) is the center and r is the radius
# returns a list of the values of t where the cubic intersects the circle

# Returns a list of collision tuples. 
#   Each collision has the form (intersectionPts, ptOfInterest, circle)
#   Where intersectionPts are the two pts that intersect the circle. Only values
#       between the intersection points are inside the circle
#   ptOfInterest is the point between a par of intersection points that is closest 
#       the the center of the circle

def cubicSplineCircleCollisions(csx, csy, tVals, circle):
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

    # Loop through all segments of the given cubic spline
    for i in xrange(len(tVals) - 1):
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

            # If the knot is inside the segment, don't reset the critical points
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

# Helper function for main. Returns a list of circle tuples
# Each circle tuple is of the form ((h, k), r) where (h, k) is the center and r is
#   the radius
def makeRandomCircles(numCircles, wpx, wpy):
    import random

    # Put given waypoints into a list of (x, y) points
    pts = []
    for i in xrange(len(wpx)):
        pt = (wpx[i], wpy[i])
        pts.append(pt)

    # Generate Random Obstacles
    count = 0
    circles = []

    # Create n random, non-overlapping circles
    while (count <= numCircles):
        # Random center and radius
        h = random.uniform(0.0, 1300.0)
        k = random.uniform(0.0, 800.0)
        r = random.uniform(30*12*0.0254, 300*12*0.0254)

        # Don't keep the circle if is swallows a waypoint or is inside another 
        #   circle
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

        if key:
            continue

        # Add circle to plot and list of circles
        circles.append(((h, k), r))
        count += 1

    return circles

#----------------------------------------------------------------------------------#

# Given 3 lists, a list for each of the x, and y coordinates of waypoints, as well
#   as an init spacing for the parameter. Initial spacing must be stricly increasing
# Lists need to be of the same length
# Pass a costFunction that evaluates the cost of the spline

# If successful, returns (t_opt, True) where t_opt is the optimal spacing
# If unsuccessful, returns (t, False) where t is the original spacing
def optimizeParameterSpacing(wpx, wpy, t, costFunc):
    def evalSpacing(x):
        # The first two parameters are fixed
        # Any scalar multiple of the spacing results in the same spline
        spacing = [t[0], t[1]]

        # Add the other (n-2) terms to t
        for param in x:
            # x contains the difference between the current and previous index
            # Exponential is used to ensure the value is strictly positive
            # This ensures that t is strictly increasing
            spacing.append(spacing[-1] + np.exp(param))

        csx = CubicSpline(spacing, wpx)
        csy = CubicSpline(spacing, wpy)

        # Return some cost of the spline
        return costFunc(csx, csy)
    #--------------------------------#

    x0 = []
    for i in xrange(len(t)):
        if (i == 0 or i == 1):
            continue

        # Difference between the current     
        diff = t[i] - t[i-1]
        x0.append(np.log(diff))

    # x is now an array representing the natural log of the difference between 
    #   non-fixed values of t
    x0 = np.array(x0)

    # To aquire the respective spacing, simply exponentiate an element of x and add 
    #   it to the previous element in the spacing
    # This allows for the elements of x to take on any real value while ensuring
    #   that the respective spacing is strictly increasing
    # TO DO: probably give some kind of jacobian to make this faster
    opt_obj = minimize(evalSpacing, x0, method = 'Nelder-Mead', tol = 1e-8)

    # Determine if the optimization was successful (it should be in all cases)
    if (opt_obj.success):
        x_opt = opt_obj.x

        # The first two parameters are fixed
        # Any scalar multiple of the spacing results in the same spline
        t_opt = [0, t[1]]

        # Add the other (n-2) terms to t
        for param in x_opt:
            # x contains the difference between the current and previous index
            # This ensures that t is monotomically increasing
            t_opt.append(t_opt[-1] + np.exp(param))

        return (t_opt, True)

    # If unsucessful, deal with it. Life isn't fair
    else:
        return (t, False)

#----------------------------------------------------------------------------------#

# csx is the cubic spline of x as a function of t
# csy is the cubic spline of y as a function of t

# tVals is the list of parametric points serving as the knots for the cubic spline

# poly is of the form [pt-1, pt-2, ... pt-n] where each pt represents a vertice of
#   the competition boundary

# Returns a list of collision tuples. 
#   Each collision has the form (intersectionPts, ptOfInterest, circle)
#   Where intersectionPts are the two pts that intersect the circle. Only values
#       between the intersection points are inside the circle
#   ptOfInterest is the point between a par of intersection points that is closest 
#       the the center of the circle

def cubicSplinePolygonCollisions(csx, csy, tVals, poly):
    # Returns true if point is inside or on the boundary of the given polygon, 
    #   else false
    def isInsidePoly(poly, pt):
        # Define a machine precision value for floating point comparisons
        epsilon = sys.float_info.epsilon

        # Returns the euclidean distance between two points
        def dist(pt1, pt2):
            return np.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)

        # Returns True if given point is on line and False if given point is not on line
        # Enpoints are considered to be on the line
        def isPtOnLine(segment, pt):
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
            x1, y1 = segment[0]
            x2, y2 = segment[1]

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
            if (np.isclose((y3 - b), (slope * x3), 0, epsilon)):
                # Check that the point is within the bounds of the segment
                if isInBounds(pt):
                    return True
                else:
                    return False

            else:
                return False

        # Returns the intersection point of the two given line segments
        # Returns None if there is no intersection point on the segments
        def intersection(segment1, segment2):
            # Returns True if the point is between the bounds of both segments and False
            #   otherwise
            def isInBounds(pt):
                # Because it is floating point numbers, we have to check if the number
                #   is inside very close to being inside the interval
                # numpy.isclose() does this check

                # If the intersection is within the x interval of the first segment
                if (pt[0] >= xInterval1[0]) and (pt[0] <= xInterval1[1]):
                    pass
                
                elif (np.isclose(pt[0], xInterval1[0], 0, epsilon)
                        or np.isclose(pt[0], xInterval1[1])):
                    pass
                
                else:    
                    #print "intersection outside of x bounds of first segment"
                    return False

                # If the intersection is within the x interval of the second segment
                if (pt[0] >= xInterval2[0]) and (pt[0] <= xInterval2[1]):
                    pass

                elif (np.isclose(pt[0], xInterval2[0], 0, epsilon)
                        or np.isclose(pt[0], xInterval2[1])):
                    pass

                else:
                    return False

                # If the intersection is within the y interval of the first segment
                if (pt[1] >= yInterval1[0]) and (pt[1] <= yInterval1[1]):
                    pass

                elif (np.isclose(pt[1], yInterval1[0])
                        or np.isclose(pt[1], yInterval1[1])):
                    pass

                else:
                    return False

                # If the intersection is within the y interval of the second segment
                if (pt[1] >= yInterval2[0]) and (pt[1] <= yInterval2[1]):
                    pass

                elif (numpy.isclose(pt[1], yInterval2[0])
                        or numpy.isclose(pt[1], yInterval2[1])):
                    pass

                else:
                    return False

                return True

            # Assumes lines intersect at most one time (not infinitely many points)

            # segment1 & segment2 are given in the form (x1, y1), (x2, y2) and (x3, y3), 
            #   (x4, y4) respectively
            x1, y1 = segment1[0]
            x2, y2 = segment1[1]
            
            x3, y3 = segment2[0]
            x4, y4 = segment2[1]

            # Find the x intervals of the line segments
            xInterval1 = [min(x1, x2), max(x1, x2)]
            xInterval2 = [min(x3, x4), max(x3, x4)]

            yInterval1 = [min(y1, y2), max(y1, y2)]
            yInterval2 = [min(y3, y4), max(y3, y4)]

            # There is no intersection point if the x and y intervals of the lines 
            #   don't overlap
            if (xInterval1[1] <= xInterval2[0]) and (xInterval2[1] <= xInterval1[0]):
                return None
            
            if (max(y1,y2) <= min(y3,y4)) and (max(y3,y4) <= min(y1,y2)):
                return None

            # Calculate the slopes of the lines
            if (x1 - x2) == 0:
                slope1 = np.inf
            else:
                slope1 = (y1 - y2) / (x1 - x2)
            
            if (x3 - x4) == 0:
                slope2 = np.inf
            else:
                slope2 = (y3 - y4) / (x3 - x4)

            # Calculate the (possibly theoretical) y intercept of both line segments
            b1 = y1 - (slope1 * x1)
            b2 = y3 - (slope2 * x3)

            # There is no intersection if the lines are parallel
            if (slope1 == slope2):
                return None

            # Since the slope of the first line is infinite, and the lines interset at 
            #   most once, we know the slope of the other line isn't infinite.
            # By reaching this point of the method, we also know that both the x and y
            #   intervals of the line segments overlap
            # We also knnow that (x1 == x2) in order for the slope to be infinite
            # The following is logically concluded:
            if (slope1 == np.inf):
                xIntersection = x1
                yIntersection = b2 + slope2 * x1
                
                if (isInBounds((xIntersection, yIntersection))):
                    return (xIntersection, yIntersection)

                return None

            # The similarly for if slope2 is infinite
            if (slope2 == np.inf):
                xIntersection = x3
                yIntersection = b1 + slope1 * x3
                
                if (isInBounds((xIntersection, yIntersection))):
                    return (xIntersection, yIntersection)

                return None

            # In order to intersect the following must be true:
            #   yIntersection = slope1 * xIntersection + b1
            #   yIntersection = slope2 * xIntersection + b2
            # Therefore:
            #   slope1 * xIntersection + b1 = slope2 * xIntersection + b2
            xIntersection = (b2 - b1) / (slope1 - slope2)
            yIntersection = b1 + slope1 * xIntersection
            # The xIntersection must be between both the given x intervals
            
            if (isInBounds((xIntersection, yIntersection))):
                    return (xIntersection, yIntersection)

            return None

        # Create a ray that extends "infinitely" in one direction
        #   this ray cannot intersect the vertices of the polygon
        # If this ray intercepts the polygon an odd number of times, it is inside 
        #   the polygon. Otherwise it is outside the polygon

        # Check if point is on line of polygon. It cannot be strictly inside if so
        for i in range(0, len(poly)):
            segment = (poly[i], poly[((i+1) % len(poly))])
            
            if (isPtOnLine(segment, pt)):            
                return True

        # The farthest distance between points in a polygon will be to a vertex
        # bigDistance is an amount larger than this
        # Therefore it acts as an "infinite" distance
        bigDistance = 0
        for vertex in poly:
            bigDistance = max(bigDistance, dist(vertex, pt))

        bigDistance += 1 + pt[0]

        # Create a ray that extends from the point and does not intersect any of the
        #   the vertices. If the ray intersects a vertex, increment the y value of
        #   the other endpoint and check again
        ray = (pt, (bigDistance, 0))
        hitsVertex = 1

        while (hitsVertex):
            newEndPt = (ray[1][0], (ray[1][1] + 1))
            ray = (pt, newEndPt)

            for vertex in poly:
                if (isPtOnLine(ray, vertex)):
                    hitsVertex = 1
                    break
                else:
                    hitsVertex = 0

        # Count the number of intersetions with the polygon
        count = 0
        for i in range(0, len(poly)):
            segment = (poly[i], poly[((i+1)%len(poly))])

            # self.intersection returns the intersection point if it exists, or None
            count += 0 if (intersection(ray, segment) == None) else 1

        # If the count is odd, it is inside the polygon
        if (count % 2):
            return True

        else:
            return False

    # Equation of a line:
    #   y = m * x + k
    # x is defined in terms of t by csx
    # y is defined in terms of t by csy

    # collisions is the list to return
    collisions = []

    intersections = []

    distToInitPt = np.linalg.norm([(csx(tVals[0]) - poly[0][0]), 
            (csy(tVals[0]) - poly[0][1])])

    distToFinPt = np.linalg.norm([(csx(tVals[-1]) - poly[0][0]), 
            (csy(tVals[-1]) - poly[0][1])])

    resetCritPts = True

    # Loop through all segments of the given cubic spline
    for i in xrange(len(tVals) - 1):
        # tX is a list of the coefficients of the equation x = At^3 + Bt^2 + Ct + D
        # tY is a list of the coefficients of the equation y = at^3 + bt^2 + ct + d 
        tX = [csx.c[0][i], csx.c[1][i], csx.c[2][i], csx.c[3][i]]
        tY = [csy.c[0][i], csy.c[1][i], csy.c[2][i], csy.c[3][i]]

        # Loop through all lines of the polygon
        for j in xrange(len(poly)):
            x = (poly[j-1][0], poly[j][0])
            y = (poly[j-1][1], poly[j][1])

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

            print i, " -th cubic spline: "
            print "Line segment: x: ", x, "\ty: ", y

            print roots

            raw_input(" ")

            # Weed out the garbage points
            for root in roots:
                # Doesn't count if it is an imaginary root
                if not np.isreal(root):
                    continue

                # The root must be between the start and end of this segment
                if (root >= 0 and root <= (tVals[i+1] - tVals[i])):
                    intersections.append(root.real + tVals[i])

            # The end pts count as intersection points if they are inside the poly
            #   and we are analysing that particular segment
            if ((i == 0) and isInsidePoly(poly, (tX[0], tY[0]))):
                # This avoids the edge case where the endpoint is on the poly
                if (tVals[0] not in intersections):
                    intersections.append(tVals[0])

            elif ((i == (len(tVals) - 2)) and isInsidePoly(poly, (tX[0], tY[0]))): 
                # This avoids the edge case where the endpoint is on the poly
                if (tVals[-1] not in intersections):
                    intersections.append(tVals[-1])

            # Sort the intersection points to make manipulations easier
            intersections.sort()

            print intersections


            # np.roots() to solve for 0's of coeff
            # Only keep intersection points that are within the bounds of the spline
            # Only keep intersection points that are within the bounds the line 
            #   segment polygon

            # Calculate pts of interest if intersection points exist
            #   Rotate abt origin
            #   Find critical points wrt t' (rotated parametric function independent 
            #       variable)
            #   Rotate (t', y') back into (x, y) plane
            #   Find value of t that aligns with pt of interest


#----------------------------------------------------------------------------------#

# csx and csy are the parametric cubic splines with knots at the points in t
# collisions is a list of collision objects
# Collision object is of the form (intersectionPts, ptOfInterest, circle)
#   or (intersectionPts, ptOfInterest, line)

def fixIntersections(csx, csy, t, collisions):
    # TO DO
    return 0

#----------------------------------------------------------------------------------#

def main():
    # Initialize Figure
    fig, ax = plt.subplots()
    plt.gca().set_xlim([0,  1300])
    plt.gca().set_ylim([0, 800])

    # Initialize 'given' waypoints
    wpxInit = [600, 250, 100, 1000, 1050, 800, 750]
    wpyInit = [500, 600, 375, 390, 650, 650, 200]
    plt.plot(wpxInit, wpyInit, 'x', label ='data', color = (0,0,0,1))

    # Makes parameter spacing to be constant
    n = len(wpxInit)
    t = np.arange(n)

    csx = CubicSpline(t, wpxInit)
    csy = CubicSpline(t, wpyInit)

    # Plot parametric cubic splines
    s = 0.01
    tSpace = np.arange(t[0], t[n-1] + s, s)
    plt.plot(csx(tSpace), csy(tSpace))

    poly = [(250, 500), (900, 400), (800, 600), (200, 600)]

    xVals = list()
    yVals = list()
            
    for pt in poly:
        x, y = pt

        xVals.append(x)
        yVals.append(y)
    
    x, y = poly[0]
    xVals.append(x)
    yVals.append(y)
    plt.plot(xVals, yVals)
    plt.show(block=False)
    cubicSplinePolygonCollisions(csx, csy, t, poly)


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



    plt.show()

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
Half    3) Fix that
        4) Find regions that have high curvature
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