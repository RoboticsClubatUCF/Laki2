from math import atan, atan2, cos, sin, sqrt, tan, pi
import numpy as np
from scipy.optimize import minimize

# comparisons were made against google earth and https://www.whoi.edu/marine/ndsf/cgi-bin/NDSFutility.cgi?form=0&from=LatLon&to=XY
# Also https://www.movable-type.co.uk/scripts/latlong.html

# latOrig and latPos are in degrees North (All of America is in N)
# lonOrig and lonPos are in degrees West (All of America is in W)
# ___Orig are the origin values
# ___Pos are the requested values

# Returns a result of (x, y) where x, y are the (East, North) distances
# Has trouble converging for points near the poles

def gps_to_xy(latOrig, lonOrig, latPos, lonPos):
    # Convert to radians
    latOrig *= pi / 180
    lonOrig *= pi / 180
    latPos *= pi / 180
    lonPos *= pi / 180

    # Define Constants------------------------------------------#
    # Radius at equator in meters (WGS-84)
    a = 6378137.0
    
    # Flattening of the ellipsoid (WGS-84)
    # (Earth is non-spherical)
    f = 1 / 298.257223563 
    
    # Radius at pole in meters (WGS-84)
    b = (1 - f) * a  

    # Change in Longitude
    deltaLon = lonPos - lonOrig  

    # Latitudes
    lats = [latOrig, latPos]

    # ???
    tU = [(1 - f) * tan(lats[i]) for i in range(0, 2)]    
    cU = [1 / (sqrt(1 + (tU[i]) ** 2)) for i in range(0, 2)]
    sU = [tU[i] * cU[i] for i in range(0, 2)]

    # Initialize a value for the change in longitude on the aux-sphere
    auxLon = deltaLon

    # Repeat iterative method until the longitude between points on the 
    #   aux-shere converges
    for i in range(0, 1000):
        sinAuxLon = sin(auxLon)
        cosAuxLon = cos(auxLon)

        # Calculate Sigma, the arc length between points on the non-sphere
        # ssig and csig are intermediate values
        ssig = (cU[1] * sinAuxLon) ** 2 
        ssig += (cU[0] * sU[1] - cU[1] * sU[0] * cosAuxLon) ** 2
        ssig = ssig ** 0.5
        csig = sU[0] * sU[1] + cU[0] * cU[1] * cosAuxLon
        sigma = atan(ssig / csig)

        # Calculate alpha, the azimuth angle at the equator
        salpha = (cU[0] * cU[1] * sinAuxLon) / ssig
        salpha2 = salpha ** 2
        calpha2 = 1 - salpha2
        c2sigm = csig - (2 * sU[0] * sU[1]) / calpha2
        C = (f / 16) * calpha2 * (4 + f * (4 - 3 * calpha2))
        
        # auxLon is lambda, the longitude between the points on the non-sphere
        auxLon_old = auxLon
        
        auxLon = deltaLon + (1 - C) * f * salpha * (sigma + 
                        C * ssig * (c2sigm + C * csig * (-1 + 2 * (c2sigm ** 2))))
        
        # Repeat until the convergence error is below 1e-12 (approx 0.06mm)
        if abs(auxLon - auxLon_old) < 1e-12:
            break

    # Compute new parameters from parameters found above
    u2 = calpha2 * (((a ** 2) - (b ** 2)) / (b ** 2))

    # Some mathemagic series expansion coefficients
    A = 1 + (u2 / 16384) * (4096 + u2 * (-768 + u2 * (320 - 175 * u2)))
    B = (u2 / 1024) * (256 + u2 * (-128 + u2 * (74 - 47 * u2)))
    
    # Arc length between pts on the aux-sphere
    deltaSig = B * ssig * (c2sigm + (B / 4) * 
            (csig * (-1 + 2 * (c2sigm ** 2)) - (B / 6) * c2sigm * 
            (-3 + 4 * (ssig ** 2)) * (-3 + 4 * (c2sigm ** 2))))
    
    # Geodesic between the 2 given points on the ellipsoid
    s = b * A * (sigma - deltaSig)
    
    # Alpha is the forward azimuth angles at the points
    alpha = [atan2((cU[0] * sU[1] - sU[0] * cU[1] * cosAuxLon), cU[1] * sinAuxLon),
            atan2(sU[1] * cU[0] * cosAuxLon - cU[1] * sU[0], (cU[0] * sinAuxLon))]

    # Finally, calculate the x and y distances
    x = s * cos(alpha[0])
    y = s * sin(alpha[1])

    return [x, y]

#----------------------------------------------------------------------------------#

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


def main():
    latBase = 38.14046388888889
    lonBase = -76.43536111111112

    latPt = 38.145002777777776
    lonPt = -76.41499722222223
    print (gps_to_xy(latBase, lonBase, latPt, lonPt))

if (__name__ == '__main__'):
    main()