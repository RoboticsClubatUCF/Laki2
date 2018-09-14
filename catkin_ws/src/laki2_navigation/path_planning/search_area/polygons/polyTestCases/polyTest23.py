# Differentiably close intersection points causes issues with hashing

import polygon
import copy

def main():
    initBounds = [[(-0.230979309934631, -0.1), (-0.0, -0.1), (-0.0, -0.05), 
                    (0.05, -0.05), (0.05, -0.0), (0.29, 0.0), 
                    (0.0354415588, -0.2545584412), (-0.1054415588, -0.3127012595)]]


    poly = polygon.polygon(initBounds)

    rect =  [(0.019865145634921, -0.04588437630505), 
            (-0.21111416429971, -0.145884376305049), 
            (-0.250844455569552, -0.05411562369495), 
            (-0.019865145634921, 0.04588437630505)]


    showDog = copy.deepcopy(initBounds)
    showDog.append(rect)
    print "showDog: ", showDog
    showDog = polygon.polygon(showDog)
    showDog.show(1, 1)

    poly.clip(rect)
    poly.show(1, 1)


if (__name__ == "__main__"):
    main()