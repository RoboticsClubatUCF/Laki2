from __future__ import division
import numpy
import math
import collections
import inspect
import copy
import sys
import matplotlib.pyplot as plt

# define a machine precision value for floating point comparisons
epsilon = sys.float_info.epsilon

# define a constant of which to round when requested
precision = 4

# Toggle to print debugging statements
debug = 0

# Super debuggers
isInsideDebug = 0
isPtOnLineDebug = 0

# Defines a polygon class
# Polygon is composed of a list of single polygons
# Each single polygon must be be non-self intersecting, and have no holes for 
#   operations to be valid
class polygon:
    def __init__(self, pts=list()):
        self.polygons = list()

        # If there is no given input, there is an error somewhere. But not the fault
        #   of this script
        if (len(pts) == 0):
            print "{0} {1}".format("No points given\nPolygon init requires a",
                    "list of polygons or a list for a single polygon's points\n")
            return

        # If a list is given, it is assumed to be a list of polygons
        if (type(pts[0]) == type(self.polygons)):
            self.polygons = copy.deepcopy(pts)

        # Otherwise it is assumed to be a list of points of a single polygon
        else:
            self.polygons.append(list(copy.deepcopy(pts)))

        self.standardize()


    # returns the euclidean distance between two points
    def dist(self, pt1, pt2):
        return math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)


    # Returns True if given point is on line and False if given point is not on line
    # Enpoints are considered to be on the line
    def isPtOnLine(self, segment, pt):
        if isPtOnLineDebug:
            print "Entering isPtOnLine"
            raw_input("")
            print "segment: ", segment
            print "pt: ", pt

        # Returns True if the point is between the bounds of the segment and False
        #   otherwise
        def isInBounds(pt):
            # Because it is floating point numbers, we have to check if the number
            #   is inside very close to being inside the interval.
            #   numpy.isclose() does this check

            # If the intersection is within the x interval of the segment
            if (pt[0] >= xInterval[0]) and (pt[0] <= xInterval[1]):
                pass
            
            elif (numpy.isclose(pt[0], xInterval[0], 0, epsilon)
                    or numpy.isclose(pt[0], xInterval[1])):
                pass
            
            else:    
                if isPtOnLineDebug:
                    print "out of x bounds"
                return False

            # If the intersection is within the y interval of the segment
            if (pt[1] >= yInterval[0]) and (pt[1] <= yInterval[1]):
                pass

            elif (numpy.isclose(pt[1], yInterval[0], 0, epsilon)
                    or numpy.isclose(pt[1], yInterval[1])):
                pass

            else:
                if isPtOnLineDebug:
                    print "out of y bounds"
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
            slope = float("inf")
        else:
            slope = (y1 - y2) / (x1 - x2)
            if isPtOnLineDebug:
                print "slope: ", slope

        # special case when slope of line is infinite
        if slope == float("inf"):
            if isPtOnLineDebug:
                print "slope == inf"

            # if the slope is infinite, but the x values aren't the same, it is not 
            #   on the line
            if not (numpy.isclose(x1, x3, 0, epsilon)):
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
        if (numpy.isclose((y3 - b), (slope * x3), 0, epsilon)):
            # Check that the point is within the bounds of the segment
            if isInBounds(pt):
                if isPtOnLineDebug:
                    print "returning True"
                return True
            else:
                if isPtOnLineDebug:
                    print "out of bounds, returning false"
                return False

        else:
            if isPtOnLineDebug:
                print "pt not on line"
            return False


    # Returns the intersection point of the two given line segments
    # Returns None if there is no intersection point on the segments
    def intersection(self, segment1, segment2):
        # Returns True if the point is between the bounds of both segments and False
        #   otherwise
        def isInBounds(pt):
            # Because it is floating point numbers, we have to check if the number
            #   is inside very close to being inside the interval
            # numpy.isclose() does this check

            # To Do: This is a potential issue if the lines intersect at a point
            #   outside one of the segments but very close to the endpoints
            # The only time floating points come into play is when they intersect
            #   at an endpoint

            # If the intersection is within the x interval of the first segment
            if (pt[0] >= xInterval1[0]) and (pt[0] <= xInterval1[1]):
                pass
            
            elif (numpy.isclose(pt[0], xInterval1[0], 0, epsilon)
                    or numpy.isclose(pt[0], xInterval1[1])):
                pass
            
            else:    
                #print "intersection outside of x bounds of first segment"
                return False

            # If the intersection is within the x interval of the second segment
            if (pt[0] >= xInterval2[0]) and (pt[0] <= xInterval2[1]):
                pass

            elif (numpy.isclose(pt[0], xInterval2[0], 0, epsilon)
                    or numpy.isclose(pt[0], xInterval2[1])):
                pass

            else:
                #print "intersection outside of x bounds of second segment"
                return False

            # If the intersection is within the y interval of the first segment
            if (pt[1] >= yInterval1[0]) and (pt[1] <= yInterval1[1]):
                pass

            elif (numpy.isclose(pt[1], yInterval1[0])
                    or numpy.isclose(pt[1], yInterval1[1])):
                pass

            else:
                #print "intersection outside of y bounds of first segment"
                return False

            # If the intersection is within the y interval of the second segment
            if (pt[1] >= yInterval2[0]) and (pt[1] <= yInterval2[1]):
                pass

            elif (numpy.isclose(pt[1], yInterval2[0])
                    or numpy.isclose(pt[1], yInterval2[1])):
                pass

            else:
                #print "intersection outside of y bounds of second segment"
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
            slope1 = float("inf")
        else:
            slope1 = (y1 - y2) / (x1 - x2)
        
        if (x3 - x4) == 0:
            slope2 = float("inf")
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
        if (slope1 == float("inf")):
            xIntersection = x1
            yIntersection = b2 + slope2 * x1
            
            if (isInBounds((xIntersection, yIntersection))):
                return (xIntersection, yIntersection)

            return None

        # The similarly for if slope2 is infinite
        if (slope2 == float("inf")):
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


    # Returns true if the point is inside this polygon, else false
    # args takes (point) or (point, polygonPoints)
    # point is the point in question
    # polygonPoints is a list of single polygons
    def isInside(self, *args):
        # if only one input is given, it should be a point, and the request to know 
        #   if the pointt is inside of this polygon
        if (len(args) == 1):
            for singlePoly in self.polygons: 
                if (self.isInsideSingle(singlePoly, args[0])):
                    return True
        
        # if two inputs are given, they should be a polygon and pt in that order.
        #   The request to know if the point given is inside the given polygon
        elif (len(args) == 2):
            #print "args[0]: ", args[0]
            #print "args[1]: ", args[1]
            for singlePoly in args[0]:
                if (self.isInsideSingle(singlePoly, args[1])):
                    return True
        
        # otherwise there is an issue with the input
        else:
            print "Error with size of input to isInside"

        # if none of the singlePolygons analyzed returned True, it is not in any of
        #   them. So return False
        return False


    # Returns true if point is strictly inside the given polygon, else false
    # singlePoly is a single polygon
    def isInsideSingle(self, singlePoly, pt):
        # Create a ray that extends "infinitely" in one direction
        #   this ray cannot intersect the vertices of the polygon
        # If this ray intercepts the polygon an odd number of times, it is inside 
        #   the polygon. Otherwise it is outside the polygon

        if isInsideDebug:
            print "entering isInsideSingle"

        # Check if point is on line of polygon. It cannot be strictly inside if so
        for i in range(0, len(singlePoly)):
            segment = (singlePoly[i], singlePoly[((i+1)%len(singlePoly))])
            if isInsideDebug:
                print "segment: ", segment
            
            if (self.isPtOnLine(segment, pt)):
                if isInsideDebug:
                    print "intersecting polygon, not inside"
                
                return False

        # The farthest distance between points in a polygon will be to a vertex
        # bigDistance is an amount larger than this
        # Therefore it acts as an "infinite" distance
        bigDistance = 0
        for vertex in singlePoly:
            bigDistance = max(bigDistance, self.dist(vertex, pt))

        bigDistance += 1 + pt[0]

        # Create a ray that extends from the point and does not intersect any of the
        #   the vertices. If the ray intersects a vertex, increment the y value of
        #   the other endpoint and check again
        ray = (pt, (bigDistance, 0))
        hitsVertex = 1

        while (hitsVertex):
            newEndPt = (ray[1][0], (ray[1][1] + 1))
            ray = (pt, newEndPt)

            for vertex in singlePoly:
                if (self.isPtOnLine(ray, vertex)):
                    hitsVertex = 1
                    break
                else:
                    hitsVertex = 0

        if isInsideDebug:
            print "ray: ", ray

        # Count the number of intersetions with the polygon
        count = 0
        for i in range(0, len(singlePoly)):
            segment = (singlePoly[i], singlePoly[((i+1)%len(singlePoly))])
            if isInsideDebug:
                print "segment: ", segment
            
            # self.intersection returns the intersection point if it exists, or None
            count += 0 if (self.intersection(ray, segment) == None) else 1
            if isInsideDebug:
                print "count: ", count

        # If the count is odd, it is inside the polygon
        if (count % 2):
            if isInsideDebug:
                print "intersects an even number of times. Is not inside"
            return True

        else:
            if isInsideDebug:
                print "intersects an odd number of times. Is inside"
            return False


    # Modifies self.polygons to remove the area of the given rectangle from all
    #   applicable polygons
    def clip(self, rectangle):
        newPolygons = list()

        rectArea = self.areaSingle(rectangle, True)

        if (numpy.isclose(rectArea, 0, 0, epsilon)):
            return

        # signed area of a clockwise polygon is negative
        if (rectArea > 0):
            rectangle = list(reversed(rectangle))

        self.noColinear()

        if debug:
            print "self.polygons: ", self.polygons

        # clip each individual polygon seperately
        for singlePoly in self.polygons:
            newPoly = self.clipSingle(rectangle, singlePoly)
            for newSinglePolygon in newPoly:
                newPolygons.append(newSinglePolygon)

        self.polygons = newPolygons
        if debug:
            print "self.polygons, pre Standardization: ", self.polygons

        self.standardize()
        if debug:
            print "self.polygons, post standardization: ", self.polygons


    # Returns a list of the polygon(s) with the area of the rectangle removed
    # Rectangle is assumed to be a rectangle polygon, but should be able to be 
    #   any polygon. It is the area to be clipped away from the polygon (clip)
    # singlePoly is a single polygon to be clipped (subject)
    def clipSingle(self, rectangle, singlePoly):
        originalPoly = set(singlePoly)
        originalRect = set(rectangle)

        # Must find all intersection pts and insert them into both the rectangle
        #   and polygon between the points that created the lines that intersect
    
        # Insert all intersection points into the polygon
        # We iterate through all the lines of the polygon, so all intersections with
        #   for a single line in the polygon are found in one iteration
        # Intersections with the each line of the rectangle is a bit harder
        # rectIntersect is a dictionary with line segments of the rectangle as keys
        # Insert all those intersections after iterating through the polygon
        modifiedPoly = list()
        rectIntersects = dict()
        intersectionPts = set()

        for i in range(0, len(singlePoly)):
            if debug:
                print "\n"

            # Find all the intersections with the polygon
            lineIntersections = list()
            alphas = list()

            # polyEndPt wraps around the list once it reaches the end
            polyEndPt = (i + 1) % len(singlePoly)

            # polySegment is the line created by the current and next vertex
            polySegment = (singlePoly[i], singlePoly[polyEndPt])

            if debug:
                print "polySegment: ", polySegment

            for j in range(0, len(rectangle)):
                # rectEndPt wraps around the list once it reaches the end
                rectEndPt = (j+1) % len(rectangle)

                # rectSegment is the line created by the current and next vertex
                rectSegment = (rectangle[j], rectangle[rectEndPt])
                
                if debug:
                    print "rectSegment: ", rectSegment 

                # find the intersection point btwn 
                lineIntersectionPt = self.intersection(polySegment, rectSegment)

                if debug:
                    print "lineIntersectionPt: ", lineIntersectionPt

                if (lineIntersectionPt != None):
                    # Log the intersection point for future use
                    intersectionPts.add(lineIntersectionPt)

                    # alpha is dist from the first polygon vertex to intersection
                    alpha = self.dist(lineIntersectionPt, singlePoly[i])
                    
                    # add the intersection, alpha pair to the intersections list
                    lineIntersections.append((lineIntersectionPt, alpha))
                    
                    # beta is dist from the first rectangle vertex to intersection
                    beta = self.dist(lineIntersectionPt, rectangle[j])
                    
                    # rectSegIntersects is a list of the previous intersections with 
                    #   this segment of the rectangle
                    rectSegIntersects = rectIntersects.setdefault(rectSegment, 
                                                                            list())

                    # append the newly found intersection to the list
                    rectSegIntersects.append((lineIntersectionPt, beta))

                    # update the dictionary entry
                    rectIntersects[rectSegment] = rectSegIntersects

            if debug:
                print "adding to modifiedPoly: ", singlePoly[i]
            
            # need to add the current vertex to the polygon regardless of if it 
            #   intersects with the rectangle
            modifiedPoly.append(singlePoly[i])

            # Don't need to act anything if this line of the polygon doesn't 
            #   intersect the rectangle, so continue to next vertex of the polygon
            if (len(lineIntersections) == 0):
                continue

            # Sort the intersections list by alpha distances
            lineIntersections = sorted(lineIntersections, key=lambda x: x[1])

            # Add the pts to the modified polygon in order of alpha values
            for pt in lineIntersections:
                if debug:
                    print "adding to modifiedPoly: ", pt[0]
                
                modifiedPoly.append(pt[0])

                # Insert redundant pts into the intersection pts
                if numpy.isclose(singlePoly[i], pt[0], 0, epsilon).all():
                    if debug:
                        print "adding to intersectionPts: ", singlePoly[i]

                    intersectionPts.add(singlePoly[i])
                    

        # iterate thru the rectangle and add the intersection points to the modified 
        #   rectangle
        modifiedRect = list()
        for j in range(0, len(rectangle)):
            # rectEndPt wraps around the list once it reaches the end
            rectEndPt = (j+1) % len(rectangle)

            # rectSegment is the line created by the current and next vertex
            rectSegment = (rectangle[j], rectangle[rectEndPt])

            # take the points that intersect with this line of the rectangle
            rectSegIntersects = rectIntersects.setdefault(rectSegment, None)

            # add the current vertext to the new polygon
            modifiedRect.append(rectangle[j])
           
            # Don't need to do anything if this line of the polygon doesn't 
            #   intersect the rectangle, so continue to next vertex of the polygon
            if (rectSegIntersects == None):
                continue

            # sort the intersection points based on beta values
            rectSegIntersects = sorted(rectSegIntersects, key=lambda x: x[1])

            # Add the pts to the new polygon in order of alpha values
            for pt in rectSegIntersects:
                modifiedRect.append(pt[0])

        if debug:
            print "\n\nintersectionPts: ", intersectionPts
            print "\nmodifiedPoly: ", modifiedPoly, "\n"
            print "\nmodifiedRect: ", modifiedRect

        # Intersection points at vertices of either the polygon or rectangle get
        #   added multipl times, remove any of these duplicate points
        temp = list()
        for i in range(0, len(modifiedPoly)):
            if not numpy.isclose(modifiedPoly[i-1], modifiedPoly[i], 
                        0, epsilon).all():
                temp.append(modifiedPoly[i])
        modifiedPoly = temp

        temp = list()
        for i in range(0, len(modifiedRect)):
            if not numpy.isclose(modifiedRect[i-1], modifiedRect[i], 
                        0, epsilon).all():
                temp.append(modifiedRect[i])
        modifiedRect = temp

        if debug:
            print "\npost redundant pt removal modifiedPoly: ", modifiedPoly, "\n"
            print "\npost redundant pt removal modifiedRect: ", modifiedRect, "\n"

        # Reverse the ordering of the rectangle
        modifiedRect = list(reversed(modifiedRect))

        if debug:
            print "\npost reversal modifiedRect: ", modifiedRect, "\n"       

        # New Polygon Creation Rules:
        #   Traverse the modified polygon
        #   If an intersection point is reached, switch to the modified rectangle
        #   Traverse the modified rectangle starting with the intersection point 
        #       from the polygon that started the traversal in the rectangle
        #   Continue in the modified rectangle until another intersection point is 
        #       reached (loop around rectangle if the end of the list is met)
        #   When an intersection point in the rectangle is reached, switch back to 
        #       the modified polygon, starting with the intersection point that 
        #       sent the traversal back to the polygon
        #   Continue in the polygon until either anoher intersection point is 
        #       reached, or the end of the polygon is reached.
        #   If an intersection point is reached, repeat steps as done previously
        #   The end of the polygon marks the end of a new single polygon
        #   The points traversed create a new polygon
        #   Remove all points from the polygon and the rectangle traversed 
        #       during this iteration
        #   Do NOT remove intersection points
        #   Continue traversals until there are only intersection points in the 
        #       modified polygon
        #   Edge cases for intersections at vertices are explained in shape jumpers

        #---------------------------------------------------------------------------
        # Recursive helper method to traversing throught the polygon
        def shapeJumperPoly(pt_index):
            # Stop when the end of the polygon is reached
            if (pt_index >= len(modifiedPoly)):
                return

            startingIndex = pt_index

            if (startingIndex <= lastIndexUsed[0]):               
                return

            # starting from the index after the given point, iterate through the 
            #   modified polygon until an intersection point is found or the end of 
            #   the polygon list is found
            # points that are found are added to the newSinglePolygon
            # when intersection point found, go to modified rectangle at that point
            for i in range(startingIndex, len(modifiedPoly)):                
                # modifiedPoly[i] is the current point
                # Add the current point to the new polygon list
                newSinglePoly.append(modifiedPoly[i])
                lastIndexUsed[0] = i

                # Jumping can happen at intersection points
                # Jumping depends on what the next point of the rectangle is:
                #   If the next point of the rectangle is inside the polygon, jump
                #   If the next point of the rectangle would cause the traversal to 
                #       go outside the polygon at any point, don't jump
                #   If the current point and next point of the rectangle create a 
                #       line that is colinear with any line on the polygon, don't 
                #       jump

                isIntersectionPt = False
                if modifiedPoly[i] in intersectionPts:
                    isIntersectionPt = True
               
                # Because we deleted redundant points, the current point could be
                #   very close, but not the exact same as an intersection pt
                else:
                    for intersectionPt in intersectionPts:
                        if numpy.isclose(intersectionPt, modifiedPoly[i], 
                                0, epsilon).all():
                            isIntersectionPt = True

                if isIntersectionPt: 
                    # Find the first index of the current point
                    for k in range(0, len(modifiedRect)):
                        if numpy.isclose(modifiedRect[k], modifiedPoly[i], 
                                                    0, epsilon).all():
                            break
                    
                    # The previous for loop basically does this but accounts for
                    #   floating point inaccuracies:
                    # k = modifiedRect.index(modifiedPoly[i])

                    currPt = modifiedRect[k]
                    nextPt = modifiedRect[ (k + 1) % len(modifiedRect) ]
                    
                    # halfPt is the pt halfway between the current point and the
                    #   next point. Because lines are linear, it is the point on the
                    #   line halfway between the two
                    # It is better to do calculations with the half point because
                    #   the next point could be on the vertex of the rectangle or
                    #   something else funky. So knowing if the half point is inside
                    #   outside or intersecting the polygon tells us about the line
                    #   more than the next point
                    # (half is arbitrary, could be any value 0 < x < 1
                    halfPt = ((currPt[0] + nextPt[0])/2, (currPt[1] + nextPt[1])/2)

                    # If the next point is neither inside the polygon nor 
                    #   intersecting the polygon, it must be outside the polygon
                    if self.isInside([singlePoly], halfPt):
                        shapeJumperRect((k + 1) % len(modifiedRect) )
                        break

                        
        # Recursive helper method to traversing throught the rectangle
        def shapeJumperRect(pt_index):            
            startingIndex = pt_index

            # Starting from the index after the given point, iterate through the 
            #   modified rectangle until an intersection point is found. Wrapping 
            #   around the modified rectangle as necessary
            # Points that are found are added to the newSinglePolygon
            # When intersection point is found, return to the polygon at that point

            # It is garanteed that there will be an even number of intersection
            #   points that are not also vertices of the polygon
            # Therefore, we must always return to the polygon from the rectangle

            for i in range(0, len(modifiedRect)):
                wrappingIndex = (startingIndex + i) % len(modifiedRect)

                newSinglePoly.append(modifiedRect[wrappingIndex])
                
                # Flag an intersetion boolean if this point is an intersection
                isIntersectionPt = False
                if modifiedRect[wrappingIndex] in intersectionPts:
                    isIntersectionPt = True
                
                # Because we deleted redundant points, the current point could be
                #   very close, but not the exact same as an intersection pt
                else:
                    for intersectionPt in intersectionPts:
                        if numpy.isclose(intersectionPt, 
                                modifiedRect[wrappingIndex], 0, epsilon).all():
                            isIntersectionPt = True

                if isIntersectionPt:
                    # Find the first index of the current point
                    for k in range(0, len(modifiedPoly)):
                        if numpy.isclose(modifiedPoly[k], 
                                    modifiedRect[wrappingIndex], 0, epsilon).all():
                            break
                    
                    # The previous for loop basically does this but accounts for
                    #   floating point inaccuracies:
                    # k = modifiedRect.index(modifiedPoly[i])

                    currPt = modifiedPoly[k]
                    nextPt = modifiedPoly[ (k + 1) % len(modifiedPoly) ]
                    
                    # halfPt is the pt halfway between the current point and the
                    #   next point. Because lines are linear, it is the point on the
                    #   line halfway between the two
                    # It is better to do calculations with the half point because
                    #   the next point could be on the vertex of the rectangle or
                    #   something else funky. So knowing if the half point is inside
                    #   outside or intersecting the polygon tells us about the line
                    #   more than the next point
                    # (half is arbitrary, could be any value 0 < x < 1
                    halfPt = ((currPt[0] + nextPt[0])/2, (currPt[1] + nextPt[1])/2)

                    # If the next line would go inside the rectangle, continue
                    if self.isInside([rectangle], halfPt):
                        continue
                    else:
                        shapeJumperPoly((k + 1) % len(modifiedPoly))
                        break                       
        
        #---------------------------------------------------------------------------

        # newPoly will be the temp store of polygons before we update self.polygons
        newPoly = list()

        # Create polygons until only intersection points remain in the modified poly
        # The condition on the while loop here will not be the exit condition. But 
        #   the exit condition is complicated and explained later
        while (len(modifiedPoly) > 0):
            # Shift polygon list to begin with a point stricty outside of the 
            #   rectangle. (all points inside were removed, so only intersection
            #   points are a concern
            for i in range(0, len(modifiedPoly)):
                if debug:
                    print "modifiedPoly[0]: ", modifiedPoly[0]

                # If the next line would go inside the rectangle, continue
                isInside = False
                if self.isInside([rectangle], modifiedPoly[0]):
                    if debug:
                        print "inside Pts"
                    isInside = True

                # Can't start with an intersection pt
                isIntersectionPt = False
                if modifiedPoly[0] in intersectionPts:
                    if debug:
                        print "intersection Pt"
                    isIntersectionPt = True
                   
                # Because we deleted redundant points, the current point could be
                #   arbitrarily close, but not the exact same as an intersection pt
                # So check if it is close enough to an intersection pt to be 
                #   considered and intersection point
                else:
                    for intersectionPt in intersectionPts:
                        if debug:
                            print "intersectionPt: ", intersectionPt

                        if numpy.isclose(intersectionPt, modifiedPoly[0], 0, 
                                    epsilon).all():
                            if debug:
                                print "intersection Pt"
                            isIntersectionPt = True

                if isInside or isIntersectionPt:
                    modifiedPoly.append(modifiedPoly.pop(0))
                else:
                    break

            # The previous loop checked elements until it either hit a point 
            #   strictly outside the rectangle, or checked the entire list
            # If element currently first in the list is still not strictly outside 
            #   the rectangle, that means there is no point strictly outside of it
            # Therefore, the algorithm is complete and it is time to return
            if isIntersectionPt or isInside:
                return newPoly

            if debug:
                print "post shifting modifiedPoly: ", modifiedPoly

            # newSinglePoly will be the temporary store while we create each polygon
            # newSinglePoly is used by the shape jumpers
            newSinglePoly = list()

            # Shape jumpers don't append the value they are given, so this has to be
            #   done manually before starting the jumps
            newSinglePoly.append(modifiedPoly[0])
            lastIndexUsed = [0]

            if debug:
                print "jumping to poly"
            
            # Start the shape jumper at the first index of the modified poly
            shapeJumperPoly(1)

            if debug:
                print "found polygon, newSinglePoly: ", newSinglePoly

            # Remove points from modifiedPoly and modifiedRect after they are
            #   inserted into the polygons
            for pt in newSinglePoly:
                if pt not in intersectionPts:
                    for vertex in modifiedPoly: 
                        if numpy.isclose(vertex, pt, 0, epsilon).all():
                            modifiedPoly.remove(vertex)
                    
                    for vertex in modifiedRect: 
                        if numpy.isclose(vertex, pt, 0, epsilon).all():
                            modifiedRect.remove(vertex)

            if debug:
                print "\npost removal modifiedPoly: ", modifiedPoly
                print "\npost removal modifiedRect: ", modifiedRect

            newPoly.append(newSinglePoly)

        # Once only intersection points remain, return the list of polygons
        return newPoly
    

    # Breaks each polygon in self.polygons into as many convex polygons as necessary
    def makeConvex(self):
        newPolys = list()
        for singlePoly in self.polygons:
            newPoly.append(self.makeConvexSingle(singlePoly))

        self.polygons = newPolys

    # Takes a single polygon and returns a list of convex polygons that occupy the
    #   same area
    def makeConvexSingle(self, singlePoly):
        # TO DO
        return

    # Rounds all values in the polygon
    # Useful for maintaining sets when floating point error case two "identical"
    #   polygons to be seen as different
    # Default rounding place is defined at start of document as precision. Can be
    #   overridden by caller
    def rounded(self, roundPlace=precision, doSelf=False):
        newPolys = list()

        # Round every single polygon 
        for singlePolygon in self.polygons:
            newSinglePoly = list()
            # Round the x and y values of each vertex in the single polygon
            for vertex in singlePolygon:
                vertex = ((round(vertex[0], roundPlace), 
                            round(vertex[1], roundPlace)))
                newSinglePoly.append(vertex)
            newPolys.append(newSinglePoly)

        if doSelf:
            self.polygons = newPolys

        return newPolys

    def removeRedundant(self, givenPoly=None):
        # If no input is given, it is assumed to want to remove the redundancied 
        #   from this polygno
        if (givenPoly == None):
            polygons = self.polygons
        # Otherwise make the given polygon non-colinear
        else:
            polygons = givenPoly 
        
        newPolys = list()
        for singlePoly in polygons:
            newPoly = list()
            for i in range(0, len(singlePoly)):
                # The epsilon here is a lot larger because redundant points wreak 
                #   havoc
                if not numpy.isclose(singlePoly[i-1], singlePoly[i], 
                        0, 32*epsilon).all():
                    newPoly.append(singlePoly[i])
            newPolys.append(newPoly)
        
        if (givenPoly == None):
            self.polygons = newPolys
        else:
            return givenPoly


    # Removes any points in the polygon that are colinear
    # Views each vertex of the polygon. If that vertex is colinear with the line
    #   created by its neighbors it is redundant
    def noColinear(self, givenPoly=None, eps=epsilon):
        # If no input is given, it is assumed to make this polygon non-colinear
        if (givenPoly == None):
            polygons = self.polygons
        # Otherwise make the given polygon non-colinear
        else:
            polygons = givenPoly 

        # Redundant points break this process
        self.removeRedundant(polygons)

        # Store all the non-colinear points in a new list
        newPolygons = list()
        for singlePolygon in polygons:

            newSinglePoly = list()
            for i in range(0, len(singlePolygon)):
                wrapper = (i + 1) % len(singlePolygon)
                line = (singlePolygon[i-1], singlePolygon[wrapper])

                # if the current point is not colinear with the previous and next 
                #   points of this polygon, add it to the new polygon
                if not self.isPtOnLine(line, singlePolygon[i]):
                    newSinglePoly.append(singlePolygon[i])

            newPolygons.append(newSinglePoly)

        if (givenPoly == None):
            self.polygons = newPolygons
        else:
            return newPolygons 


    # Ensures that vertices of all polygons are ordered clockwise
    # Ensure that all vertices of a polygon are not colinear with their neighbors
    # Sorts polygons from least area to most
    # Shifts each individual polygon to begin with most -x point. 
    #   Ties for leftmost are broken by +y value
    def standardize(self):
        # Fix any colinearities (also fixed redundancies)
        self.noColinear(eps=epsilon)

        # Sort polygons by area
        areaPolygons = list()
        for singlePolygon in self.polygons:
            area = self.areaSingle(singlePolygon, 1)
            
            # If the polygon has zero area, leave
            if numpy.isclose(area, 0, 0, epsilon):
                continue

            # If the points are ordered counterclockwise, reverse the ordering
            if (area > 0):
                singlePolygon = list(reversed(singlePolygon))

            area = abs(area)            
            areaPolygons.append((singlePolygon, area))

        # Sorts the temp store by area        
        # The temp store is a list of tuple(singlePolygon, area)
        areaPolygons = sorted(areaPolygons, key=lambda x: x[1])
        
        # Wipe self.polygons bc we are about to repopulate it with the polygons 
        #   sorted by area
        self.polygons = list()

        # Populate self.polygons in order of area
        for polyAreaPair in areaPolygons:
            self.polygons.append(polyAreaPair[0])

        # Shifts each individual polygon to begin with the leftmost point        
        shiftedPolygons = list()
        for i in range(0, len(self.polygons)):
            singlePoly = self.polygons[i]

            # Find j, the index of the highest priority vertex
            index = 0
            for j in range(0, len(singlePoly)):
                x, y = singlePoly[j]
                if (x < singlePoly[index][0]):
                    index = j

                if ((x == singlePoly[index][0]) 
                        and (y > singlePoly[index][1])):
                    index = j

            # The following creates a new points list that is shifted forward such 
            #   that it begins with the previous index found
            shiftedPoly = list()
            for j in range(0, len(singlePoly)):
                newIndex = (j + index) % len(singlePoly)
                shiftedPoly.append(singlePoly[newIndex])

            # Make the points list the new points list
            shiftedPolygons.append(shiftedPoly)

        self.polygons = shiftedPolygons


    # prints the list of points in the polygon
    def disp(self):
        print self.polygons


    # Alternate method to plot polygon
    def show(self, doShow=0, doWait=0):
        self.plot(doShow, doWait)


    # plots a matlab-like figure of the current polygons
    def plot(self, doShow=0, doWait=0):
        fig, ax = plt.subplots()

        axes = [float("inf"), -float("inf"), float("inf"), -float("inf")]
        
        for singlePoly in self.polygons:
            xVals = list()
            yVals = list()
            
            for pt in singlePoly:
                x, y = pt

                axes = [min(axes[0], x), max(axes[1], x), 
                        min(axes[2], y), max(axes[3], y)]

                xVals.append(x)
                yVals.append(y)
            
            x, y = singlePoly[0]
            xVals.append(x)
            yVals.append(y)
            ax.plot(xVals, yVals, 'k', c=numpy.random.rand(3,1))

        
        for i in range(0, len(axes)):
            if (abs(axes[i]) == float("inf")):
                axes[i] = (-1)**(i+1)
            else:
                # shift box away from polygon by this amount
                offset =  0.1

                # fix the sign of offset
                offset *= (-1)**(i+1)

                # scale the box to be this much bigger than the polygon
                scale = 1.1

                # adjust the axes value
                axes[i] = scale * (axes[i] + offset)


        plt.axis(axes)

        if (doShow):
            plt.show(block=False)

            if (doWait):
                    print "Press Enter to Exit"
                    raw_input("")


    # Calulates the total unit area of the polygons
    def area(self):
        area = 0
        
        # Calculate the area of all polygons in self.polygons
        for singlePoly in self.polygons:
            area += self.areaSingle(singlePoly)

        return area
    

    # Calculates the area of the single given polygon
    def areaSingle(self, singlePoly, signed=False):    
        # Area of an arbitrary polygon is defined as:
        #   M_i = [x_i, x_i+1; y_i, y_i+1]
        #   A = 0.5 * (det(M_1) + det(M_2) + ... + det(M_n))
        
        area = 0
        for i in range(0, len(singlePoly)):
            # polyEndPoint wraps around the list once it reaches the end
            polyEndPt = (i + 1) % len(singlePoly)

            area += (singlePoly[i][0] * singlePoly[polyEndPt][1] -
                             (singlePoly[polyEndPt][0]) * singlePoly[i][1])
        area *= 0.5

        # Area is negative if points are ordered in a clockwise traversal
        # By default the positive area is return
        # Option of returning signed area is available by setting signed to true
        if not signed:
            return abs(area)
        else:
            return area


    # Calculates the total unit length of the perimeter of the polygons
    def perimeter(self):
        # Perimeter is the sume of the side lengths of the polygon
        perim = 0
        for singlePoly in self.polygons:
            for i in range(0, len(singlePoly)):
                # polyEndPt wraps around the list once it reaches the end
                polyEndPt = (i + 1) % len(singlePoly)

                perim += self.dist(singlePoly[i], singlePoly[polyEndPt])
        
        return perim



def main():
    """
    initBounds = [(-0.29, 0.0), (0.0, 0.0), (0.0, -0.05), (0.05, -0.05), 
                (0.1, -0.05), (0.1, 0.0), (0.29, 0.0), (0.0354, -0.2546), 
                (-0.1054, -0.3127)]
    poly = polygon(initBounds)

    rect = [(0.05, -0.1), (-0.0, -0.1), (0.0, 0.0), (0.05, -0.0)]
    rect1 = [(0.05, 0.0), (0.1, 0.0), (0.1, -0.1), (0.05, -0.1)]

    showDog = list()
    showDog.append(initBounds)
    showDog.append(rect)
    print "showDog: ", showDog
    showDog = polygon(showDog)
    showDog.show(1, 1)

    poly.clip(rect)
    poly.show(1, 1)
    """


if (__name__ == "__main__"):
    main()