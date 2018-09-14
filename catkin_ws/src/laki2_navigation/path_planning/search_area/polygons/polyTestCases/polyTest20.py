# Differentiably close intersection points causes issues with hashing

import polygon

def main():
    initBounds = [(0.05, 0.1), (0.1, 0.1), (0.1, 0.05), (0.07071068, 0.05), 
                    (0.08535534, 0.06464466)]
    poly = polygon.polygon(initBounds)

    rect = [(0.08535534, 0.13535534), (0.12071068, 0.1), (0.05, 0.02928932), (0.01464466, 0.06464466)]

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
