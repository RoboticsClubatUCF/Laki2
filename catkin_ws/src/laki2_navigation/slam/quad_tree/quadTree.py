from __future__ import division
import polygon
import math
import numpy as np
import matplotlib
import matplotlib.pyplot as plt


# Smallest length, in m, of a square than can contain the entire boundary
#   while still maintaining N as the y direction and E as the east direction
maxLen = 1332

# Gives near centimeter precision (1.02 cm)
maxDepth = 17

class quadNode:
    def __init__(self, depth = 0, location = (maxLen/2, maxLen/2), occupied = False):
        # The number of nodes above this node in the tree
        self.depth = depth

        # The (x, y) of the center of this cell
        self.location = tuple(location)

        # Node array of children
        # Begins as Null
        # Child is same as quadrant # -1
        #   (+x, +y) = 0
        #   (+x, -y) = 1
        #   (-x, -y) = 2
        #   (-x, +y) = 3
        self.children = None

        # Start a node as unoccupied
        self.occupied = occupied


    # Returns false if this node is as deep as allowed
    # Creates children if none exist
    # Returns true once this node has children
    def makeChildren(self):
        if (self.children != None):
            return True

        if (self.depth >= maxDepth):
            return False

        self.children = list()
        newDepth = self.depth + 1
        newDist = maxLen / (2**(newDepth+1))

        for i in range(0, 4):
            newLocation = list(self.location)
            
            if (i < 2):
                newLocation[0] += newDist
            else:
                newLocation[0] -= newDist

            if (i == 0 or i == 3):
                newLocation[1] += newDist
            else:
                newLocation[1] -= newDist

            newChild = quadNode(newDepth, newLocation, self.occupied)
            self.children.append(newChild)

        return True

    def isAnyChildOccupied(self):
        # If this node is occupied, return True
        if self.occupied:
            return True

        # Since this node is not occupied, and it has no children, nothing inside of 
        #   it is occupied. So return False
        if (self.children == None):
            return False

        # This node is not occupied, but check if any of its children are occupied
        for child in self.children:
            # If any of it's children are occupied, return True
            if isAnyChildOccupied(child):
                return True

        # Since neither this node nor any of its children are occupied, return False
        return False


    def deleteUnoccupiedChildren(self, doRecurse = False):
        # If nothing inside of this node is occupied delete the children
        # They are a waste of space 
        if not self.isAnychildOccupied:
            self.children = None
        
        # otherwise, recursively delete the children of this node who are have 
        #   nothing inside them 
        else:
            if doRecurse:
                for child in self.children:
                    child.deleteUnoccupiedChildren()

    def getCorners(self):
        dist = maxLen / (2**(self.depth + 1))

        corners = list()
        for i in range(0, 4):
            corner = list(self.location)
            
            if (i < 2):
                corner[0] += dist
            else:
                corner[0] -= dist

            if (i == 0 or i == 3):
                corner[1] += dist
            else:
                corner[1] -= dist

            corners.append(tuple(corner))

        return corners

    def disp(self):
        print self
        print "depth:\t", self.depth
        print "location:\t", self.location
        print "children:\t", self.children
        print "occupied\t", self.occupied 
        print "corners:\t", self.getCorners()


# Cannot remove areas as of yet. Possible options:
#   1) Better for moving objects that never leave the map: 
#       Keep a quadTree of static objects and re-insert objects whenever they move
#   2) Better for when objects enter and leave the map:
#       Make each node store the object that occupied it. Keep a set of all objects
#       currently on the map. 
#       Add a method to the quadNode class named cleanUp. The node looks at itself
#       and if the object that created it is no longer in the set, it marks itself
#       as unoccupied. Then it calls cleanUp on all of its children. This can be
#       done in parallel with deleting nodes that don't have an object anywhere
#       inside
class quadTree:
    def __init__(self):
        self.root = quadNode()

    # plots a matlab-like figure of the current quadTree
    def show(self, doShow=0, doWait=0):
        # Populates Node into Plot
        def populatePlot(node):
            # Recursively populate children
            if (node.children != None):
                for child in node.children:
                    populatePlot(child)

            else:        
                corners = node.getCorners()

                # Create a matlib path that traces the current node
                path_data = [
                    (Path.MOVETO, [corners[0][0], corners[0][1]]),
                    (Path.LINETO, [corners[1][0], corners[1][1]]),
                    (Path.LINETO, [corners[2][0], corners[2][1]]),
                    (Path.LINETO, [corners[3][0], corners[3][1]]),
                    (Path.CLOSEPOLY, [corners[0][0], corners[0][1]])
                    ]

                codes, verts = zip(*path_data)
                path = matplotlib.path.Path(verts, codes)

                if node.occupied:
                    kolor = 'black'
                else:
                    kolor = 'white'     

                # Create a matlib patch of the current node
                patch = matplotlib.patches.PathPatch(path, edgecolor = 'black', facecolor=kolor)
                
                # Add that patch to the plot
                ax.add_patch(patch)

        # Initialize plot configuration
        fig, ax = plt.subplots()        
        plt.axis([0, maxLen, 0, maxLen])
        Path = matplotlib.path.Path

        # Populate the plot with the whole tree
        populatePlot(self.root)

        if (doShow):
            plt.show(block=False)

            if (doWait):
                    print "Press Enter to Exit"
                    raw_input("")


    def markPtOccupied(self, pt):
        def markNodesOccupied(node):
            if (node.occupied == True):
                return

            if not node.makeChildren():
                node.occupied = True
                return 

            childIndex = set([0, 1, 2, 3])

            if (pt[0] > node.location[0]):
                childIndex.discard(2)
                childIndex.discard(3)
            else:
                childIndex.discard(0)
                childIndex.discard(1)

            if (pt[1] > node.location[1]):
                childIndex.discard(1)
                childIndex.discard(2)
            else:
                childIndex.discard(0)
                childIndex.discard(3)

            for index in childIndex:
                markNodesOccupied(node.children[index])

        markNodesOccupied(self.root)


    # This needs work. I think you have to preconvert the circle to a polygon
    #   Cuz it is hard to check if any part of a circle is in a square
    # if doDeleteChildren is set to True, when a node is marked occupied, it will 
    #   delete the children of that node 
    def markCircleOccupied(self, circle):
        # Helper function that marks nodes and all their children occupied
        def markNodesOccupied(node):
            print "\n"
            node.disp()

            def dist(pt1, pt2):
                return math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)

            # Helper function that returns true if the entirety of the node is 
            #   inside the polygon and False otherwise
            def isCompletelyInsideCircle(box):
                corners = box.getCorners()
                for corner in corners:
                    if (dist(corner, circle[1]) > circle[0]):
                        return False    
                return True

            # Helper function that returns true if any part of the node is inside
            #   the polygon and False otherwise
            def isSomeInsideCircle(box):
                corners = box.getCorners()
                for corner in corners:
                    # If at least one corner is inside the polygon return True
                    if (dist(corner, circle[1]) < circle[0]): 
                        return True
                return False

            # If this is already marked occupied, return
            if node.occupied:
                print "Node Already Occupied"
                return

            # If the node is completely inside the polygon, mark it occupied, delete
            #   its children and return
            if isCompletelyInsideCircle(node):
                node.occupied = True
                node.children = None
                print "Node completely inside polygon"
                return

            # markNodesOccupied is only called if we are certain at least part
            #   of the polygon will be inside the node
            # Since it is not completely inside the node, it must be partially
            #   inside of this node
            else:
                # Create children and/or check if this is the deepest node
                if not node.makeChildren():
                    # Since this is the deepestNode, if any part of the polygon
                    #   is inside of this node, mark it as occupied
                    print "Deepest Node"
                    
                    # To Do: I believe this next check is not necessary. markAreaOccupied 
                    #   is only called when we are certain some of the polygon is inside
                    #   of this node. So there is no reason to check again 
                    #if isSomeInsideCircle(node):
                    #    print "Occupied"
                    node.occupied = True
                
                # Since this is not the deepest node, recurse down the tree another
                #   layer
                else:
                    # Check all children
                    for child in node.children:
                        print "Checking child: ", child
                        child.disp()
                        corners = child.getCorners()

                        # If any pt of the singlePoly is inside the child, call 
                        #   markNodesOccupied on the child
                        for corner in corners:
                            if isSomeInsideCircle(child):
                                print "There is a pt of the polygon inside this child"
                                markNodesOccupied(child)
                                break

            node.disp()
            print "returning"


        markNodesOccupied(self.root)

    # Turns the given circle into 
    def turnCircleIntoPoly(self, radius, center, numSides):
        xOffset, yOffset = center

        pts = list()
        for i in range(0, numSides):
            xPt = (np.sin(i / numSides * 2 * np.pi) * radius) + xOffset;
            yPt = (np.cos(i / numSides * 2 * np.pi) * radius) + yOffset;
            pts.append((xPt, yPt))

        return polygon.polygon(pts)



def main():
    tree = quadTree()
    print "tree.root.location: ", tree.root.location
    tree.markPtOccupied((100, 1000))
    tree.markPtOccupied((200, 1000))
    tree.markPtOccupied((400, 800))
    tree.markPtOccupied((600, 200))
    #tree.show(0, 0)

    #crc = (100, (800, 800))
 
    #poly.disp()
    #tree.markCircleOccupied(crc)
    tree.show(1, 1)



if (__name__ == "__main__"):
    main()