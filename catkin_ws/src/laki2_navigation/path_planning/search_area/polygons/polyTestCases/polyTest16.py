# Rectangle lines B and D of rectangle intersect poly, endpoint of D intersects poly

import polygon

def main():
    initBounds = [(0, 0), (0, 4), (4, 4)]
    poly = polygon.polygon(initBounds)

    rect = [(1, 5), (3, 5), (3, 3), (1, 3)]

    showDog = list()
    showDog.append(initBounds)
    showDog.append(rect)
    print "showDog: ", showDog
    showDog = polygon.polygon(showDog)
    showDog.show(1, 1)

    poly.clip(rect)
    poly.show(1, 1)



if (__name__ == "__main__"):
    main()