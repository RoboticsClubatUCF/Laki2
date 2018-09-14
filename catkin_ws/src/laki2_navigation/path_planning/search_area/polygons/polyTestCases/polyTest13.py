# poly completely inside rectangle

import polygon

def main():
    initBounds =  [(0,4), (4, 4), (2, 0)]

    poly = polygon.polygon(initBounds)

    rect = [(-1, 5), (5, 5), (5, -1), (-1, -1)]

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