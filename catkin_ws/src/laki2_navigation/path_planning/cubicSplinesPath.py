from __future__ import division
import sys
import numpy as np
import scipy
import copy
import Polygon
from scipy.interpolate import CubicSpline, interp1d
from scipy.optimize import minimize
from scipy import integrate
from scipy.misc import derivative
import matplotlib.pyplot as plt
from matplotlib.pyplot import Circle
from matplotlib.collections import PatchCollection

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
    # TO DO: probably give some kind of jacobian to make this faster
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
# csz is the cubic spline of z as a function of t

# tVals is the list of parametric points serving as the knots for the cubic spline

def arcLength(csx, csy, csz, tVals):
    totalLength = 0

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
                lambda t: np.sqrt(fx_dt(t)**2 + fy_dt(t)**2 + fz_dt(t)**2), 
                tVals[i], tVals[i + 1], maxiter = 10000)
        
        length = length[0]

        # add the segment length to the total path length
        totalLength += length

    return totalLength

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

# TO DO: Convert from poly being a list as an input to a polygon object
# poly is a Polygon object where each pt represents a vertice of the comp boundary

# Returns a list of collision tuples. 
#   Each collision has the form (intersectionPts, ptOfInterest, polySeg)
#   Where intersectionPts are the two pts that intersect the polygon. Only values
#       between the intersection points are outside the polygon
#   ptOfInterest is the point between a par of intersection points that is farthest 
#       outside the polygon
#   polySeg is the line segment of intersection.
#       It is of the form ((x0, x1), (y0, y1))

def cubicSplinePolygonCollisions(csx, csy, tVals, poly):
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

    # Loop through all segments of the given cubic spline
    for i in range(len(tVals) - 1):
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

            isFinPtOutside = not (isPtOnLine((x, y), 
                                        (csx(tVals[-1]), csy(tVals[-1]))) 
                            or polyObj.isInside(csx(tVals[-1]), csy(tVals[-1])))

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
# csx and csy are the parametric cubic splines with knots at the points in t
# collisions is a list of collision objects (both circle and polygon)
# Collision object is of the form (intersectionPts, ptOfInterest, circle)
#   or (intersectionPts, ptOfInterest, line)

def fixIntersections(wpx, wpy, tVals, collisions):
    csx = CubicSpline(tVals, wpx)
    csy = CubicSpline(tVals, wpy)

    for collision in collisions:
        ptOfInterest = (csx(collision[1]), csy(collision[1]))
        
        # index is the index in which to insert the new point to wpx and wpy
        for index in range(len(tVals)):
            if (collision[1] < tVals[index]):
                break

        # It is a polygon collision
        if (len(collision[2][1]) > 1):
            line = collision[2]
            #slope = (line[1][0] - line[1][1]) / (line[0][0] - line[0][1])
            
            slope = (wpy[index] - wpy[index - 1]) / (wpx[index] - wpx[index - 1])

            # Slope of the line with which to adjust the intermedite waypoint
            #perpSlope = -1 / slope

            perpSlope = slope

            # Equation of a line:
            #   y = mx + k
            #k = ptOfInterest[1] - perpSlope * ptOfInterest[0]

            k = wpy[index - 1] - perpSlope * wpx[index - 1]

            newPt = copy.copy(ptOfInterest)


            # This just plots what the new spline looks like with a new 
            #   waypoint inserted
            
            # For polygon interesection, intermediate wapoint insertion seems to
            #   look good on the line between the waypoints sandwiching the intersection
            #   and not much to do with the actual ptOfInterest

            for i in range(20):
                plt.plot(collision[2][0], collision[2][1])

                t = copy.copy(tVals.tolist())
                t.insert(index, collision[1])
                
                wpxNew = copy.copy(wpx)
                #wpxNew.insert(index, ptOfInterest[0] + 10*i)
                wpxNew.insert(index, wpx[index - 1] + 5*i)

                wpyNew = copy.copy(wpy)
                #wpyNew.insert(index, ptOfInterest[1] + 10*i*perpSlope)
                wpyNew.insert(index, wpy[index - 1] + 5*i*perpSlope)

                plt.plot(wpxNew, wpyNew, 'x', label = 'data', color = (0,0,0,1))

                csx = CubicSpline(t, wpxNew)
                csy = CubicSpline(t, wpyNew)

                # Plot parametric cubic splines
                s = 0.01
                tSpace = np.arange(t[0], t[len(t)-1] + s, s)
                plt.plot(csx(tSpace), csy(tSpace))
                plt.show()






    return None

#----------------------------------------------------------------------------------#

def main():
    # Initialize Figure
    fig, ax = plt.subplots()
    plt.gca().set_xlim([0,  1300])
    plt.gca().set_ylim([0, 800])

    # Initialize 'given' waypoints
    wpxInit = [600, 250, 100, 1000, 1050, 800, 750]
    wpyInit = [500, 600, 375, 390, 650, 650, 200]
    wpzInit = [100, 100, 100, 100, 100, 100, 100]

    plt.plot(wpxInit, wpyInit, 'x', label = 'data', color = (0,0,0,1))

    # Makes parameter spacing to be constant
    n = len(wpxInit)
    t = np.arange(n)

    #t = optimizeParameterSpacing(wpxInit, wpyInit, t, arcLength)[0]

    #print (t)

    csx = CubicSpline(t, wpxInit)
    csy = CubicSpline(t, wpyInit)

    # Plot parametric cubic splines
    s = 0.01
    tSpace = np.arange(t[0], t[n-1] + s, s)
    #plt.plot(csx(tSpace), csy(tSpace))

    poly = [(250, 700), (0, 300), (1150, 100), (1100, 750)]

    xVals = list()
    yVals = list()
            
    """
    for pt in poly:
        x, y = pt

        xVals.append(x)
        yVals.append(y)
    
    x, y = poly[0]
    xVals.append(x)
    yVals.append(y)
    plt.plot(xVals, yVals)
    """

    collisions = cubicSplinePolygonCollisions(csx, csy, t, poly)

    fixIntersections(wpxInit, wpyInit, t, collisions)


    print ("collisions: ",     collisions)

    intersectionPts = []
    ptsOfInterest = []

    for collision in collisions:
        intersections, ptOfInterest, cicle = collision
        
        intersectionPts.append(intersections[0])
        intersectionPts.append(intersections[1])
        
        ptsOfInterest.append(ptOfInterest)

    plt.plot(csx(intersectionPts), csy(intersectionPts), 'o', color = 'g')
    plt.plot(csx(ptsOfInterest), csy(ptsOfInterest), 'o', color = 'y')

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