from vehicle_properties import vehicle
import numpy as np

def main():
    # Motor Data
    Kv = 320 * 2 * np.pi / 60
    Kt = 1/Kv
    motorSpec = [0.116, Kv, Kt, 7, 50, 165]

    # Propeller Data
    propDiameter = 18
    propPitch = 6.5
    propSpec = [propDiameter, propPitch]

    motorRPM = 2958

    drone = vehicle()

    print "propProperties: ", drone.propProperties(motorRPM, propSpec, 0, 0)


if (__name__ == "__main__"):
    main()