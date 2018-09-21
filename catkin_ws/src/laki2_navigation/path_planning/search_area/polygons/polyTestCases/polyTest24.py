# Differentiably close intersection points causes issues with hashing

import polygon
import copy

def main():
    initBounds =  [[(-364.28454330344675, -132.50514068970335), 
                    (-231, -7), (-185.54850266048686, -67.49155543860536)]]

    poly = polygon.polygon(initBounds)

    rect =   [(-789.9088050863579, -541.3426111907429), 
                    (-675.890204177974, 30.30934308965709), 
                    (304.79319885700045, -165.2924539668677), 
                    (190.7745979486166, -736.9444082472677)]

    showDog = copy.deepcopy(initBounds)
    showDog.append(rect)
    #print "showDog: ", showDog
    showDog = polygon.polygon(showDog)
    showDog.show(1, 1)

    poly.clip(rect)
    poly.show(1, 1)


if (__name__ == "__main__"):
    main()