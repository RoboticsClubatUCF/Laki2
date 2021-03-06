from vehicle_properties import vehicle
import numpy as np

def main():
    # Motor Data
    Kv = 300 * 2 * np.pi / 60
    Kt = 1/Kv
    motorSpec = [0.062, Kv, Kt, 7, 50, 0]

    # Propeller Data
    propSpec = (18, 6.5)

    motorRPM = 6000
    motorRPMs = [6005, 6000]
    dists = [6437.38, 4023.36]

    drone = vehicle()

    weight = drone.weight(8, 8, motorSpec, 0, 6, 18, 0)
    print (weight)

    print ("propProperties: ", drone.propProperties(motorRPM, propSpec, 0, 0))

    print ("euler speed: ", drone.eulerSpeed(weight, 8, 8, motorRPM, propSpec))


    print ("speed, alpha: ", drone.speed(weight, 8, 8, motorRPM, propSpec))

    #drone.powerConsumption(5200, motorSpec, 3.7*6, 8, propSpec, 8.02, 0.08)

    #print drone.batteryCapacity(motorRPMs, motorSpec, 8, 8, 6, propSpec, dists)

    #print drone.alpha(4.0, weight, 8, 5000, propSpec)

if (__name__ == "__main__"):
    main()