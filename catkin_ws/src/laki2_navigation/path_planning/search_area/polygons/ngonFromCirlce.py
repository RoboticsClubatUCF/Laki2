# Andrew Schroeder 7/19/18
# Script to find vertices (x,y) of an n-sided polygon that circumscribes a circle of 
#   known radius r with center at (x,y)
# A higher number n (more sides) corresponds to more accurate approximation of the circle

import math

def createPolygon(center, radius, numSides):
    xOffset, yOffset = center

    # Find internal angle of each triangle that makes up the polygon
    dtheta = math.pi * 2 / numSides

    # Find length of hypotenuse of each right triangle with angle theta/2
    h = radius / math.cos(dtheta/2)

    points = []
    theta = 0

    # Calculate x and y for each vertex of the polygon
    for i in range(numSides):
    	x = xOffset + (h * math.cos(theta))
        y = yOffset + (h * math.sin(theta))
    	points.append((x, y))
        theta += dtheta

    # Output the (x,y) coordinate pair for each polygon vertex
    return points


def main():
    createPolygon((0,0), 10, 10)

if __name__ == "__main__":
    main()