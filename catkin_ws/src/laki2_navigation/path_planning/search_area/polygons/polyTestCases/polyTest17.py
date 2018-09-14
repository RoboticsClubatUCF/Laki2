# Rect parallel lines A and B are colinear with poly for a portion
# Line C is colinear for entirety
# Rect insersects poly at 3 of rect's vertices

import polygon

def main():
    initBounds = [(-0.29, 0.0), (-0.05, 0.0), (-0.05, -0.05), (-0.0, -0.05), 
                        (0.0, 0.0), (0.29, 0.0), (0.0354, -0.2546), (-0.1054, -0.3127)]

    rect = [(0.0, 0.0), (0.0, -0.05), (-0.1, -0.05), (-0.1, 0.0)]

    poly = polygon.polygon(initBounds)

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