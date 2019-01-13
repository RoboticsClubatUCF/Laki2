#!/bin/sh

sim_vehicle.py -v ArduCopter -f gazebo-iris -L RC@UCF &
gazebo --verbose laki2_ardu.world &
roslaunch laki2_sim apm.launch