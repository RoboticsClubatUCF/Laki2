# Differentiably close intersection points causes issues with hashing

import polygon

def main():
    initBounds = [(0.05, 0.03454915), (0.06236068, 0.02527864), 
                (0.09236068, 0.06527864), (0.0618034, 0.0881966), (0.0559017, 0.1), 
                (0.1, 0.1), (0.1, 0.0), (0.05, 0.0)]

    poly = polygon.polygon(initBounds)

    rect = [(0.10185815468305834, 0.10234970781186546), 
            (0.11185815468305833, 0.032349707811865455), 
            (0.01286320531694167, 0.018207572188134513), 
            (0.002863205316941673, 0.08820757218813452)]

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
