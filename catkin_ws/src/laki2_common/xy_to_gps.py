from gps_to_xy import gps_to_xy
import numpy as np
from scipy.optimize import minimize

# latOrig are in degrees North (All of America is in N)
# lonOrig are in degrees West (All of America is in W)
# x, y are the (East, North) distances (in m) between the origin and the desired pt

# Returns a result of (lat, lon), where (lat, lon) maps to the desired (x, y)
# Has trouble converging for points near the poles
def xy_to_gps(latOrig, lonOrig, x, y):
    # Helper function to evaluate (lat, lon) guess
    # gps is of the form (latPos, lonPos)
    # evalLatLon returns the distance between the gps point given and the desired 
    #   (x, y) output
    def evalLatLon(gps):
        latPos, lonPos = gps
        xGuess, yGuess = gps_to_xy(latOrig, lonOrig, latPos, lonPos)
        return np.linalg.norm([x - xGuess, y - yGuess])

    # Create an initial guess slightly away from the origin
    initGuess = [latOrig + 0.07, lonOrig + 0.07]

    # Scipy minimizes the error between the desired (x, y) and the given (lat, lon)
    # Typical convergence is around 100-200 iterations, 1000 is a generous max
    opt_obj = minimize(evalLatLon, initGuess, method = 'Nelder-Mead', tol = 1e-6, 
            options = {'maxiter':1000})
    
    # Return the output vector of the minimize function
    return opt_obj.x


def main():
    latBase = 38.14046388888889
    lonBase = -76.43536111111112
    xInit, yInit = [1785.0829137403043, 503.6170014106975]

    print (xy_to_gps(latBase, lonBase, xInit, yInit))

if (__name__ == '__main__'):
    main()