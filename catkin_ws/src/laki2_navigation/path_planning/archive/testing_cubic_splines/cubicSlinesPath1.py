from __future__ import division
import copy
import collections
import inspect
import sys
import random
import numpy as np
import math
import scipy
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from matplotlib.pyplot import Circle
from matplotlib.collections import PatchCollection


#----------------------------------------------------------------------------------#

# csx is the cubic spline of x as a function of t
# csy is the cubic spline of y as a function of t

# tVals is the list of parametric points serving as the knots for the cubic spline

# circle is of the form [(h, k), r] where (h, k) is the center and r is the radius
# returns a list of the values of t where the cubic intersects the circle

# Note: does not return a value if the entire cubic spline is inside the circle
#   (There is technically no intersection in that case)

# Returns a list of collisions. 
#   Each collision hsa the form (tVals, intersectionPts, ptOfInterests, circle)
#   Where tVals are the beginning and end point of the segment of spline that 
#       intersects the circle
#   intersectionPts are the pts that intersect the circle
#   ptOfInterests contains a list of ptOfInterest. Each ptOfInterest is the pt 
#       between a par of intersection points that is closest the the center of the 
#       circle

def cubicSplineCircleCollisions(csx, csy, tVals, circle):
    # Equation of a circle:
    #   r^2 = (x - h)^2 + (y - k)^2
    # x is defined in terms of t by csx
    # y is defined in terms of t by csy

    # collisions is the object to return
    collisions = []

    # Loop through all segments of the given cubic spline
    for i in xrange(len(csx.x) - 1):
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

        # Find all intersection points between the circle and this segment

        intersections = []

        # Weed out the garbage points
        for root in roots:
            # Doesn't count if it is an imaginary root
            if not np.isreal(root):
                continue

            # The root must be between the start and end of this segment
            if (root > 0 and root < (tVals[i+1] - tVals[i])):
                intersections.append(root.real + tVals[i])

        # If the circle intersects with this segment of the spline, we need to find
        #   the ptOfInterest associated between each pair of intersections
        if intersections:
            intersections.sort()

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

            critPts = []
            # Weed out the garbage points
            for root in roots:
                # Doesn't count if it is an imaginary root
                if not np.isreal(root):
                    continue

                # The root must be between the start and end of this segment
                if (root > 0 and root < (tVals[i+1] - tVals[i])):
                    critPts.append(root.real + tVals[i])

            # Distance from first endpoint to center of circle
            distToCenter = np.linalg.norm([(csx(tVals[i]) - circle[0][0]), 
                    (csy(tVals[i]) - circle[0][0])])

            # If the distance from the first endpoint of this segment to the center 
            #   of the circle is less than the radius of the circle, the endpoint is 
            #   inside the circle
            # This means that the ptOfInterest associated with this collision could
            #   be on the previous spline, or at the knot
            # Each intersection point has a sister point. (except at the very end or
            #   very beginning of the spline) 

            # It may occur that the sister point is on another segment of the spline
            # Everything within done for within this if statement is because it is
            #   the special case where the sister point may be on the prev spline 
            if (distToCenter <= circle[1]):
                # If this is the first segment, the ptOfInterest cannot be on the 
                #   previous segment, and thus the closest point from the 'previous
                #   spline' is the first endpoint
                if (i == 0):
                    closestPt = tVals[0]

                # Initialize dist to be the distance of closest point from the 
                #   previous spline
                dist = np.linalg.norm([csx(closestPt) - circle[0][0], 
                        csy(closestPt) - circle[0][1]])

                # Check all revelvant critical points to find the closest one
                for critPt in critPts:
                    # If the crit pt is beyond the first intersection point, it will
                    #   not be the pt associated with this collision 
                    if (critPt > intersections[0]):
                        break

                    # Calculate the distance to this critical point
                    newDist = np.linalg.norm([csx(crit) - circle[0][0], 
                            csy(crit) - circle[0][1]]) 

                    # Compare the distance from this criticl point to the previous
                    # Keep the closer one
                    if (newDist < dist):
                        dist = newDist
                        closestPt = crit

                # Time to add the new collision to the collisions list:

                # If this is the first segment, the tVals are for the first segement
                # The intersection points will be the endpoint (not a true 
                #   intersection point) and the actual intersection point 
                # The closestPt is the one found from the critical points
                if (i == 0):
                    collisions.append((
                            (tVals[0], tVals[1]), 
                            (tVals[0], intersectionPts[0]), 
                            closestPt,
                            circle))

                # If this is not the first point, the tVals are such that they
                #   encompass the previous segment and the current segment
                # The intersection points are the lower and upper true intersection
                #   points
                # The closestPt is the one found from the critical points
                else:
                    collision = collisions[-1]

                    collisions[-1] = (
                            (tVals[i-1], tVals[i+1]), 
                            (collision[1][0], intersectionPts[0]),
                            closestPt,
                            circle)

                # Remove the first intersection point, the collision it is 
                #   associated with has been taken care of
                del intersections[0]

            # While there are still pairs of intersection points in the queue, 
            #   continue to create collisions
            while (len(intersections) > 1):
                # Initialize dist to infinity and closestPt to -1
                # This serves as a flag if there is an issue
                dist = np.inf
                closestPt = -1
                
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

                    # Otherwise, finf the distance from this crit point to the 
                    #   center of the circle
                    newDist = np.linalg.norm([csx(crit) - circle[0][0], 
                            csy(crit) - circle[0][1]]) 

                    # Compare it to the previous best and update if necessary
                    if (newDist < dist):
                        dist = newDist
                        closestPt = crit

                    collisions.append((
                            (tVals[i], tVals[i+1]), 
                            (intersections[0], intersections[1]),
                            closestPt,
                            circle))

                # Delete the leading pair of intersection points
                del intersections[0]
                del intersections[0]

            # The previous loop ends when there are 1 or 0 intersection points 
            #   remaining in the queue. If there is 1, the sister point is on the 
            #   next spline segment
            if (len(intersections) == 1):
                # Initialize dist to be the distance of the final endpoint
                dist = np.linalg.norm([csx(tVals[-1]) - circle[0][0], 
                        csy(tVals[-1]) - circle[0][1]])

                # Initialize the closestPt to be the endpoint
                closestPt = tVals[-1]

                # Check all revelvant critical points to find the closest one
                for critPt in critPts:
                    # If the crit pt is before the remaining intersection point, 
                    #   it will not be the pt associated with this collision 
                    if (critPt < intersections[0]):
                        continue

                    # Calculate the distance to this critical point
                    newDist = np.linalg.norm([csx(crit) - circle[0][0], 
                            csy(crit) - circle[0][1]]) 

                    # Compare the distance from this criticl point to the previous
                    # Keep the closer one
                    if (newDist < dist):
                        dist = newDist
                        closestPt = crit

                # If this is the last segment: 
                #   Set the the bounds on tVal to be the the start and end of the 
                #       last segment
                #   Set the intersection points to be the actual intersection point 
                #       and the endpoint (not a true intersection point)
                #   Set the closestPt to be the closest critical point
                if (i == len(tVals)):
                    collisions.append((
                            (tVals[-2], tVals[-1]),
                            (intersections[0], tVals[-1]),
                            closestPt,
                            circle))

                # Otherwise put the key information so the next iteration can grab
                #   what is necessary
                else:
                    collisions.append((
                            (tVals[i-1], tVals[i+1]),
                            (intersections[0], None),
                            closestPt,
                            circle))

    return collisions

#----------------------------------------------------------------------------------#

# csx is the cubic spline of x as a function of t
# csy is the cubic spline of y as a function of t

# tVals is the list of parametric points serving as the knots for the cubic spline

# pt is of the form (h, k) where (h, k) is the point of interest

# returns a list of length (tVals - 1). Each element of the list is a value of the 
#   parameter that minimizes the distance to the point within that spline

def minDistFromCubicToPt(csx, csy, tVals, pt):
    # Equation of a circle:
    #   r^2 = (x - h)^2 + (y - k)^2
    # x is defined in terms of t by csx
    # y is defined in terms of t by csy

    # Smallest dist from a pt to the curve minimizes r within the valid range of t
    # Resulting function will be a sixth order polynomial
    # The minimums of the function exist where the polynomial's derivative is 0

    closeEncounters = []

    for i in xrange(len(csx.x) - 1):
        tX = [csx.c[0][i], csx.c[1][i], csx.c[2][i], csx.c[3][i]]
        tY = [csy.c[0][i], csy.c[1][i], csy.c[2][i], csy.c[3][i]]

        # Combine constant terms from the circle and cubic
        # Inessense, modify the D and c terms to translate the center of the circle 
        #   to (0, 0)
        tX[3] -= pt[0]
        tY[3] -= pt[1]

        # Now, the equation can be of the form:
        #   r^2 = (tX)^2 + (tY)^2
        # The derivative in its expanded form is:
        #   2 * r * dr/dt = f(t)
        # Where f(t) is some function of t
        # We are interested in where r is at a minimum, so where dr/dt is 0
        # Since 2 and r will always be positive, we can ignore them and use the eqn:
        #   dr/dt = 6 * (A^2 + a^2)t^5 + 
        #           5 * (2AB + 2ab)t^4 + 
        #           4 * (2AC + B^2 + 2ac + b^2)t^3 +
        #           3 * (2AD + 2BC + 2ad + abc)t^2 + 
        #           2 * (2BD + C^2 + 2bd + c^2)t^2 +
        #           1 * (2CD + 2cd)

        coeff = [6 * (tX[0]**2 + tY[0]**2),
                5 * (2 * tX[0] * tX[1] + 2 * tY[0] * tY[1]),
                4 * (2 * tX[0] * tX[2] + tX[1]**2 + 2 * tY[0] * tY[2] + tY[1]**2),
                3 * (2 * tX[0] * tX[3] + 2 * tX[1] * tX[2] + 
                    2 * tY[0] * tY[3] + 2 * tY[1] * tY[2]),
                2 * (2 * tX[1] * tX[3] + tX[2]**2 + 2 * tY[1] * tY[3] + tY[2]**2), 
                1 * (2 * tX[2] * tX[3] + 2 * tY[2] * tY[3])]

        roots = np.roots(coeff)

        critPts = [tVals[i+1], tVals[i]]
        for root in roots:
            # Doesn't count if it is an imaginary root
            if not np.isreal(root):
                continue

            if (root > 0 and root < (tVals[i+1] - tVals[i])):
                critPts.append(root.real + tVals[i])

        dist = np.inf
        for crit in critPts:
            newDist = np.linalg.norm([csx(crit) - pt[0], csy(crit) - pt[1]]) 

            if (newDist < dist):
                dist = newDist
                closest = crit

        closeEncounters.append(closest)

    return closeEncounters

#----------------------------------------------------------------------------------#
def main():
    # Initialize Figure
    fig, ax = plt.subplots()
    plt.gca().set_xlim([0,  1300])
    plt.gca().set_ylim([0, 800])

    # Initialize 'given' waypoints
    initWPx = [250, 100, 1000, 1050, 800, 750]
    initWPy = [600, 375, 390, 650, 650, 200]
    plt.plot(initWPx, initWPy, 'x', label ='data', color = (0,0,0,1))

    # Put given waypoints into a list of (x, y) points
    pts = []
    for i in xrange(len(initWPx)):
        pt = (initWPx[i], initWPy[i])
        pts.append(pt)

    # Generate Random Obstacles
    count = 0
    circles = []

    while (count < 10):
        # Random center and radius
        h = random.randint(0, 1300)
        k = random.randint(0, 800)
        r = random.randint(30, 100)

        # Don't keep the circle if is swallows a waypoint
        key = False
        for pt in pts:
            if (np.linalg.norm((pt[0] - h, pt[1] - k)) < (r * 1.1)):
                key = True
                break

        if key:
            continue

        # Add circle to plot and list of circles
        circle = Circle((h,k), r, facecolor='r')
        circles.append(((h, k), r))
        ax.add_patch(circle)
        count += 1

    # Starting Point will have some slope dependent on vehicle heading
    slope = (initWPy[0] - initWPy[1]) / (initWPx[0] - initWPx[1])
    slopeY = slope
    slopeX = 1.0

    if (initWPy[1] < initWPy[0]):
        slopeY *= -1
        slopeX *= -1

    # Same dy/dx for any c, used to clamp cubic in direction of vehicle heading
    c = 1
    slopeY *= c
    slopeX *= c

    # Create parametric cubic spline
    t = [0]
    for i in xrange(len(wpx)):
        if (i == 0):
            continue

        t.append(t[-1] + np.linalg.norm([wpx_init[i] - wpx_init[i-1], 
                wpy_init[i] - wpy_init[i-1]]))
    
    n = len(wpx)
    t = np.arange(n)

    n = len(t)
    csx = CubicSpline(t, wpx_init, bc_type = ((1, slopeY), 'not-a-knot'))
    csy = CubicSpline(t, wpy_init, bc_type = ((1, slopeX), 'not-a-knot'))

    # Plot parametric cubic splines
    s = 0.01
    tSpace = np.arange(t[0], t[n-1] + s, s)
    plt.plot(csx(tSpace), csy(tSpace))

    # Find intersection points (collisions with no fly zones)
    # Find points of interest (pts farthest inside no fly zone,
    #   nudge these points outside no fly zones)

    # To be populated by all intersection points
    intersectionPts = []
    
    # Points most interior of circle
    ptsOfInterest = []

    # Tuples of intersected circle and related pts of interest
    collisions = []


    # To Do: turn this into a function outright
    # Find the intersection points and pts of interest for each circle
    for circle in circles:
        intersections = doesCubicIntersectCircle(csx, csy, t, circle)

        # Intersection points come out sorted
        intersectionPts += intersections

        nearestPts = minDistFromCubicToPt(csx, csy, t, circle[0])

        ptOfInterest = []
        for pt in nearestPts:
            for i in xrange(len(intersections)):
                if (i % 2):
                    continue

                if (pt > intersections[i] and pt < intersections[i + 1]):
                    ptOfInterest.append(pt)
        
        ptsOfInterest += ptOfInterest
        
        if ptOfInterest:
            collisions.append((circle, ptsOfInterest))

    # TO DO: instead of just having the circle and ptsOfInterst, include the 
    #   intersection points with the circle as well. Don't just keep adding extra
    #   points willy nilly. Add an extra point, move on to the next ptOfInterest. If
    #   at any time a future extra point causes a new intersection near the extra 
    #   point, push the extra point out a little further

    # Make a line crossing through pt of interest and circle center
    # Numerically solve for when it no longer intersects
    # Do this for all intersection points, until no intersection points exist
    while(len(collisions)):
        circle, ptsOfInterest = collisions[0]

        if (len(ptsOfInterest) > 1):
            ptOfInterst = ptsOfInterst(len(ptsOfInterest) % 2)
        else:
            ptOfInterest = ptsOfInterest

        # Find the segment of the whole spline that contains the pt of interest
        # Create a line thru the center of the circle and the point
        # Put an extra waypoint to nudge the new waypoint outside the circle
        # The new waypoint should follow along the line
        # Numerically solve for a value just outside the circle
        # Re-run doesCubicIntersectCircle and minDistFromCubicToPt for all circles
        # Repeat loop


    """
    # Plot intersection points and points of interest
    plt.plot(csx(intersectionPts), csy(intersectionPts), 'o', color = 'g')
    plt.plot(csx(ptsOfInterest), csy(ptsOfInterest), 'o', color = 'y')
    """

    plt.show()




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

if __name__ == "__main__":
    main()