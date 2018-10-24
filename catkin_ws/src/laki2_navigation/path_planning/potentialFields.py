import numpy as np
import matplotlib.pyplot as plt

# All circles should have the same strength at the circumference
# Polygon edges should all have same strength


# circle is of the form ((h, k), r) where (h, k) is the center and r is
# pos is of the form (x, y)

# Force decreases quadratically with distance

# TO DO:
#   Convert this to the force from a ring inside the same plane

# returns a vector force in the form (Fx, Fy)
def obstacleForce(circle, pos):
    distVector = [pos[0] - circle[0][0], pos[1] - circle[0][1]]
    dist = np.linalg.norm(distVector)

    # Strength at the perimeter of the circle: (subject to tuning)
    perimStrength = 100

    # Force decreases quadratically
    # F = sourceStrength / dist^2

    # F(r) = strength
    # Therefore:
    #   C = perimStrength * r^2

    sourceStrength = perimStrength * circle[1]**2

    # Magnitude of the force on the robot
    force = sourceStrength / dist**2

    # Convert to a vector force
    # tan(theta) = opp/adj = y/x
    theta = np.arctan2(distVector[1]/distVector[0])
    xForce, yForce = force*np.cos(theta), force*np.sin(theta)
    
    return (xForce, yForce)


# goal is of the form (x, y)
# pos is of the form (x, y)

# Force increases logarithmically with distance

# returns a vector force in the form (Fx, Fy)
def goalForce(goal, pos):
    distVector = [pos[0] - circle[0][0], pos[1] - circle[0][1]]
    dist = np.linalg.norm(distVector)

    goalStrength = 1

    # Magnitude of the force on the robot
    force = goalStrength * np.log(dist)

    # Convert to a vector force
    # tan(theta) = opp/adj = y/x
    theta = np.arctan2(distVector[1]/distVector[0])
    xForce, yForce = force*np.cos(theta), force*np.sin(theta)
    
    return (xForce, yForce)

# poly is of the form [pt1, pt2, ...]
#   where each pt is of the form (x, y)
# pos is of the form (x, y)

# Model the position as a positive point charge
# Model each segment of the polygon as a uniform line charge
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

    # Arbitrary field strength parameter (subject of scaling)
    strength = 1

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

        # Distance from intersection point to endpoints
        a = np.linalg.norm([intersectionPt[0] - poly[i][0], intersectionPt[1] - poly[i][1] ])
        b = np.linalg.norm([intersectionPt[0] - poly[i-1][0], intersectionPt[1] - poly[i-1][1]])

        # Distance from intersection point to position
        z = np.linalg.norm([intersectionPt[0] - pos[0], intersectionPt[1] - pos[1]])

        # Theoretical electrical field strength of a line charge at point pos
        if (z != 0):
            elecStr = strength / z * (b/((z**2 + b**2)**0.5) + a/((z**2 + a**2)**0.5))

        else:
            elecStr = np.inf
            return (np.inf, np.inf)

        # Force acts in directions of vector from midpoint of line segment to pos
        midPoint = [(poly[i-1][0] + poly[i][0])/2, (poly[i-1][1] + poly[i][1])/2]

        # Angle of the force in x and y
        theta = np.arctan2((pos[1] - midPoint[1]), (pos[0] - midPoint[0]))

        # Add force from this segment to the total forces
        forceX += elecStr * np.cos(theta)
        forceY += elecStr * np.sin(theta)

    # iterate for all corners of the polygon
    for corner in poly:     
        dist = np.linalg.norm([pos[0] - corner[0], pos[1] - corner[1]])

        # Angle of the force in x and y
        theta = np.arctan2((pos[1] - corner[1]), (pos[0] - corner[0]))

        forceX += 500*strength * np.cos(theta) / dist**2
        forceY += 500*strength * np.sin(theta) / dist**2

    return (forceX, forceY)

def main():
    # Comp Boundary converted to XY
    poly = [(100, 100), (1200, 100), (1200, 1300), (100, 1300)]

    fig, ax = plt.subplots()

    """
    # Plot the circles
    ax = plt.gca()
    for circle in circles:
        circle = Circle(circle[0], circle[1], facecolor='r')
        ax.add_patch(circle)
    """

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

    X, Y = np.meshgrid(np.arange(0, 1200, 50), np.arange(0, 1300, 50))
    
    U = np.ndarray(shape = (len(X), len(X[0])))
    V = np.ndarray(shape = (len(X), len(X[0])))

    for i in range(len(X)):
        for j in range(len(X[0])):
            U[i][j], V[i][j] = polygonForce(poly, (X[i][j], Y[i][j]))

    q = ax.quiver(X, Y, U, V)

    ax.quiverkey(q, X=0.3, Y=1.1, U=0.1,
                 label='Quiver key, length = 0.1', labelpos='E')

    plt.show()

if __name__ == "__main__":
    main()