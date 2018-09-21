# Rectangle perpendicular sides of rectangle colinear with polygon

import polygon

def main():
    initBounds = [(0, 0), (0, 3), (3, 3)]
    poly = polygon.polygon(initBounds)

    rect = [(0,3), (1,3), (1,2), (0,2)]

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