# Differentiably close intersection points causes issues with hashing

import polygon

def main():
    initBounds = [(0.0, 0.1), (0.1, 0.1), (0.1, 0.0), (0.0, 0.0)]

    poly = polygon.polygon(initBounds)

    rect = [(3.061616997868383e-18, 0.05), (0.05, 0.05), 
            (0.049999999999999996, -0.05), (-6.938893903907228e-18, -0.05)]


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
