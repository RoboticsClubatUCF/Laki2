#!/usr/bin/env python

import threading
from pymavlink import mavutil
import rospy, mavros
import math, smach, smach_ros
import time


''' to stop the 'no datalink' failsafe when running the simulator, likely not necessary for hardware
    from: http://discuss.px4.io/t/failsafe-enabled-no-datalink-when-flying-around-using-mavros-services/6816/2'''

class MavHeartbeatThread(threading.Thread):

    def __init__(self, interval=1.):
        super(MavHeartbeatThread, self).__init__()
        self.interval = interval
        self.running = False

    def run(self):
        self.running = True
        self.mavlink_connection = mavutil.mavlink_connection('0.0.0.0:14550', autoreconnect=True, baud=57600)
        self.mavlink_connection.wait_heartbeat()
        while self.running:
            self.mavlink_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_GENERIC, 0, 0, 0)
            time.sleep(self.interval)

    def stop(self):
        self.running = False


def main():

    rospy.init_node('mav_data',anonymous=True)

    heartbeat = MavHeartbeatThread()
    heartbeat.run()    

if __name__ == '__main__':
    main()        