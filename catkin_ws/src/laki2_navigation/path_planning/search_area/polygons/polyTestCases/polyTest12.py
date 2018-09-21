# Rectangle == Poly

import polygon

def main():
    initBounds =  [(0,4), (4, 4), (2, 0)]

    poly = polygon.polygon(initBounds)

    rect =  [(0,4), (4, 4), (2, 0)]

    showDog = list()
    showDog.append(initBounds)
    showDog.append(rect)
    print "showDog: ", showDog
    showDog = polygon.polygon(showDog)
    showDog.show(1, 1)

    poly.clip(rect)
    poly.show(1, 1)
    print poly.polygons

if (__name__ == "__main__"):
    main()