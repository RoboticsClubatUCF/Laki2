# Most basic intersection of rectangle and polygon

import polygon

def main():
    initBounds = [(-0.29, 0.0), (-0.25, 0.0), (-0.25, -0.05), (-0.0, -0.05), (0.0, 0.0), 
                                (0.29, 0.0), (0.0354, -0.2546), (-0.1054, -0.3127)]
    poly = polygon.polygon(initBounds)

    rect = [(0.1, 0.1), (0.15, 0.1), (0.15, -0.05), (0.1, -0.05)]

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