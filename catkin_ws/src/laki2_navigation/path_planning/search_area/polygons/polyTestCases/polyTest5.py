# Rectangle completely outside polygon

import polygon

def main():
    initBounds = [(0, 0), (0, 3), (3, 3)]
    poly = polygon.polygon(initBounds)

    rect = [(4,4), (5,4), (5,5), (4,5)]

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