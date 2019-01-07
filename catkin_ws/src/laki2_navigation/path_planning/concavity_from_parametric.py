import numpy as np
from scipy.interpolate import CubicSpline
from scipy import interpolate


def better_spline_to_concavity(csx, csy, tVals, t):
        i = 0

        for k in range(len(tVals)):
                if tVals[k] > t:
                        i = k - 1

        tX = [csx.c[0][i], csx.c[1][i], csx.c[2][i], csx.c[3][i]]
        tY = [csy.c[0][i], csy.c[1][i], csy.c[2][i], csy.c[3][i]]
        dydt = np.polyder(tY)
        dxdt = np.polyder(tX)

        # quotient rule
        top_eq = np.polysub(np.polymul(np.polyder(dydt), dxdt), np.polymul(np.polyder(dxdt), dydt))
        bottom_val = np.polyval(dxdt, t) ** 2
        quotient_of_dydx = (float(np.polyval(top_eq, t)/ bottom_val))

        return (quotient_of_dydx / np.polyval(dxdt, t))







# to be deleted from here to...

"""
def quotient_rule(dxdt, dydt, t):
        top_eq = np.polysub(np.polymul(np.polyder(dydt), dxdt), np.polymul(np.polyder(dxdt), dydt))
        bottom_val = np.polyval(dxdt, t) ** 2


        # testing the top
        print("left:")
        print(np.polymul(np.polyder(dydt), dxdt))

        print("right:")
        print(np.polymul(np.polyder(dxdt), dydt))

        print("those subtracted:")
        print(top_eq)

        print("top eqation evaluated at t:")
        print(np.polyval(top_eq, t))
        print "\n"


        # testing the bottom
        print("dx/dt:")

        print dxdt

        print "eq2 at t:"

        print np.polyval(dxdt, t)

        print ("bottom evaluated at t:")

        print bottom_val

        print ("%.5f", float(np.polyval(top_eq, t) / bottom_val))


        # testing the final outputs
        print("%.15f" % (float(np.polyval(top_eq, t)) / bottom_val))

        print("%.3f" % (154.0/6561))
        print("\n")



        return float(np.polyval(top_eq, t)) / bottom_val


def spline_to_concavity(csx, csy, t): # add t-vals
        tX = [csx[0], csx[1], csx[2], csx[3]]           # add the i's and c's back in. ex: from csx[0] to csx.c[0][i]
        tY = [csy[0], csy[1], csy[2], csy[3]]           # add the i's and c's back in. ex: from csy[0] to csy.c[0][i]
        dydt = np.polyder(tY)
        dxdt = np.polyder(tX)
        print np.polyval(dxdt, t)
        return float(quotient_rule(dxdt, dydt, t)) / np.polyval(dxdt, t)



wpxInit = [746.90, 1019.25, 390.52, 78.60, 204.4, 673.6]
wpyInit = [1108.4, 1024.40, 155.47, 391.6, 612.7, 338.4]
t = np.arange(len(wpxInit))
csx = CubicSpline(t, wpxInit)
csy = CubicSpline(t, wpyInit)

"""


# here




