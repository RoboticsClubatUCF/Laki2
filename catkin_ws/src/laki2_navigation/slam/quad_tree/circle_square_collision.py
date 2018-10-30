# Andrew Schroeder
# 8/30/2018

# Script to tell if any part of a circle intersects with any part of a rectangle 
# This script is limited to rectangles that are aligned with the x and y axes

import math
import numpy as np

# circle is of form [(h, k), r], where (h, k) is the (x, y) of the center pt
# rect   is of form [x_max, x_min, y_max, y_min]

def isRectInCircle(circle, rect):
    # Check if circle center is inside square
    if (xmin <= h) and (h <= xmax) and (ymin <= k) and (k <= ymax):
        return True

    # Check if dist from any rect corner to circle center is <= circle radius
    if ((math.sqrt((xmax - h)**2 + (ymax - k)**2) <= r) or 
            (math.sqrt((xmin - h)**2 + (ymax - k)**2) <= r) or
            (math.sqrt((xmin - h)**2 + (ymin - k)**2) <= r) or
            (math.sqrt((xmax - h)**2 + (ymin - k)**2) <= r)):
        return True

    # Check the lines of the rectangle to see if they intersect with the circle
    # This is done by checking the pts on the circle with the max and min x and y
    # (This only works bc the rectangles are always aligned with the x and y axes)

    # Top line (y = ymax)
    if ((h <= xmax) and (h >= xmin) and ((k - r) <= ymax) and ((k - r) >= ymin)):
	   return True

    # Bottom line (y = ymin)
    elif ((h <= xmax) and (h >= xmin) and ((k + r) <= ymax) and ((k + r) >= ymin)):
        return True

    # Right line (x = xmax)
    elif (((h - r) <= xmax) and ((h - r) >= xmin) and (k <= ymax) and (k >= ymin)):
        return True

    # Left line (x = xmin)
    elif (((h + r) <= xmax) and ((h + r) >= xmin) and (k <= ymax) and (k >= ymin)):
        return True

    else
        return False