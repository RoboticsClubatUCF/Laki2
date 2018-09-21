# Tests diagonal rectangle and floating point inaccuracies when finding points in a 
# list

import polygon

def main():
    initBounds = [(-0.29, 0.0), (-0.15, 0.0), (-0.15, -0.05), (-0.1, -0.05), 
                (-0.1, -0.1), (-0.0, -0.1), (-0.0, -0.05), (0.05, -0.05), 
                (0.05, -0.0), (0.29, 0.0), (0.0354, -0.2546), (-0.1054, -0.3127)]
    poly = polygon.polygon(initBounds)

    rect = [(0.0354, 0.0354), (0.0707, 0.0), (0.0, -0.0707), (-0.0354, -0.0354)]

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