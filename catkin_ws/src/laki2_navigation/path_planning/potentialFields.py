import numpy as np
import matplotlib.pyplot as plt
from cubicSplinesPath import *

#----------------------------------------------------------------------------------#

# All circles should have the same strength at the circumference
# Polygon edges should all have same strength


# circle is of the form ((h, k), r) where (h, k) is the center and r is
# pos is of the form (x, y)

# Force decreases quadratically with distance

# returns a vector force in the form (Fx, Fy)
def obstacleForce(circle, pos):
    distVector = [pos[0] - circle[0][0], pos[1] - circle[0][1]]
    dist = np.linalg.norm(distVector) - circle[1]

    # Strength at the perimeter of the circle: (subject to tuning)
    circleStrength = 10

    # Magnitude of the force on the robot
    force = circleStrength / dist**2

    # Convert to a vector force
    # tan(theta) = opp/adj = y/x
    theta = np.arctan2(distVector[1], distVector[0])
    forceX, forceY = force*np.cos(theta), force*np.sin(theta)
    
    return np.array([forceX, forceY])

#----------------------------------------------------------------------------------#

# goal is of the form (x, y)
# pos is of the form (x, y)

# Force is constant and directed towards the goal

# returns a vector force in the form (Fx, Fy)
def goalForce(goal, pos):
    distVector = [goal[0] - pos[0], goal[1] - pos[1]]
    dist = np.linalg.norm(distVector)

    goalStrength = 1

    # Magnitude of the force on the robot
    force = goalStrength #* np.log(dist)

    # Convert to a vector force
    # tan(theta) = opp/adj = y/x
    theta = np.arctan2(distVector[1], distVector[0])
    forceX, forceY = force*np.cos(theta), force*np.sin(theta)
    
    return np.array([forceX, forceY])

#----------------------------------------------------------------------------------#

# poly is of the form [pt1, pt2, ...]
#   where each pt is of the form (x, y)
# pos is of the form (x, y)

# Model the position as a positive point charge
# Quadratically decreasing force is perpendicular to line segments when intersection 
#   of perpendicular line is inside bounds of line segment. Otherwise force acts 
#   from closest endpoint and decays quadratically
# Net force on point charge is force from polygon
def polygonForce(poly, pos):
    # Helper Function
    # Finds the intersection between two lines
    # a1 and a2 are the endpoints of the first line
    # b1 and b2 are the endpoints of the second line
    def segIntersect(line1, line2):
        xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
        ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1]) #Typo was here

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(xdiff, ydiff)

        d = (det(*line1), det(*line2))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div
        return (x, y)

    #------------------------------------------------------------------------------#
        
    # Returns True if the point is between the bounds of the segment and False
    #   otherwise
    def isInBounds(seg, pt):
        # Define a machine precision value for floating point comparisons
        # Theoretically: sys.float_info.epsilon
        # In reality there are tolerance stack ups
        epsilon = 1 / 2**16

        # segment is given in the form (x1, y1), (x2, y2)
        x1, x2 = seg[0][0], seg[1][0]
        y1, y2 = seg[0][1], seg[1][1]
        
        # Find the x and y intervals of the line segments
        xInterval = [min(x1, x2), max(x1, x2)]
        yInterval = [min(y1, y2), max(y1, y2)]

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

    #------------------------------------------------------------------------------#

    # Arbitrary field strength parameter (subject of scaling)
    strength = 50

    # Total sum of forces in X and Y
    forceX = 0
    forceY = 0

    # Iterate for all lines of the polygon
    for i in range(len(poly)):     
        # If the perpendicular is not a vertical line
        if ((poly[i-1][1] - poly[i][1]) != 0):
            perpSlope = -(poly[i-1][0] - poly[i][0]) / (poly[i-1][1] - poly[i][1])
            newPt = (pos[0] + 1, pos[1] + perpSlope)

        else:
            perpSlope = np.inf
            newPt = (pos[0], pos[1] + 1)

        # Intersection point of line and perpendicular that intersects points
        intersectionPt = segIntersect((poly[i-1], poly[i]), (pos, newPt))

        if isInBounds((poly[i-1], poly[i]), intersectionPt):
            # Distance from intersection point to position
            z = np.linalg.norm([intersectionPt[0] - pos[0], intersectionPt[1] - pos[1]])

            if (z == 0):
                continue

            # Theoretical electrical field strength of a inf line charge at pos
            elecStr = strength / z**2
            
            # Angle of the force in x and y
            theta = np.arctan2((pos[1] - intersectionPt[1]), 
                        (pos[0] - intersectionPt[0]))

            # Add force from this segment to the total forces
            forceX += elecStr * np.cos(theta)
            forceY += elecStr * np.sin(theta)

        else:
            dist = np.linalg.norm([pos[0] - poly[i][0], pos[1] - poly[i][1]])
            # Theoretical electrical field strength of a point charge at pos
            elecStr = strength / dist**2
            
            # Angle of the force in x and y
            theta = np.arctan2((pos[1] - poly[i-1][1]), 
                        (pos[0] - poly[i-1][0]))

            # Add force from this segment to the total forces
            forceX += elecStr * np.cos(theta)
            forceY += elecStr * np.sin(theta)

    return np.array([forceX, forceY])

#----------------------------------------------------------------------------------#

def main():
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

    fig, ax = plt.subplots()

    circles = makeRandomCircles(30, [700], [700], poly)

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

    X, Y = np.meshgrid(np.arange(0, 1250, 8), np.arange(0, 1300, 8))
    
    U = np.ndarray(shape = (len(X), len(X[0])))
    V = np.ndarray(shape = (len(X), len(X[0])))

    maxVal = -np.inf
    for i in range(len(X)):
        for j in range(len(X[0])):
            pos = (X[i][j], Y[i][j])
            U[i][j], V[i][j] = polygonForce(poly, pos) + goalForce((700, 700), pos)

            for circle in circles:
                cirU, cirV = obstacleForce(circle, pos)
                U[i][j] += cirU
                V[i][j] += cirV

            maxVal = max(maxVal, np.sqrt(U[i][j]**2 + V[i][j]**2))

    for i in range(len(X)):
        for j in range(len(X[0])):
            if (np.sqrt(U[i][j]**2 + V[i][j]**2) > 1.2):
                theta = np.arctan2(V[i][j], U[i][j])
                U[i][j] = 1.2*np.cos(theta)
                V[i][j] = 1.2*np.sin(theta)


    q = ax.quiver(X, Y, U, V)

    ax.quiverkey(q, X=0.3, Y=1.1, U=0.5,
                 label='Quiver key, length = 0.5', labelpos='E')

    plt.show()

if __name__ == "__main__":
    main()