# Rectangle in hole just cut by other rect

import polygon

def main():
    initBounds =  [(-1, 3), (1, 3), (6, 3), (5, 0)]

    poly = polygon.polygon(initBounds)

    rect = [(1, 3), (4, 3), (4, 2), (1, 2)]

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