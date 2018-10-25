#!/usr/bin/env python

from collections import deque

import rospy, mavros
import math, smach, smach_ros
from mavros_msgs.srv import CommandBool, SetMode, ParamGet, WaypointPush, WaypointClear
from mavros_msgs.msg import State, RCIn, Waypoint, WaypointList
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix

import gps_converter as gps

class WP:

	def __init__(self, x, y, z):

		self.x = x
		self.y = y
		self.z = z

class Coord:

	def __init__(self, latitude, longitude, altitude=0):
		self.latitude = latitude
		self.longitude = longitude
		self.altitude = altitude

#totally unnecessary class to make terminal output pretty
class TextColors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'		

# just holds the copter in Standby state, doesn't do anything
class Standby(smach.State):
	
	def __init__(self):
		smach.State.__init__(self, outcomes=['toMISSION','exit_flight'])

	def execute(self, userdata):
	
		while not rospy.is_shutdown():

			return 'toMISSION'

			rospy.spin()	


class Mission(smach.State):
	
	def __init__(self):
		smach.State.__init__(self,outcomes=['toSTANDBY','exit_flight'])

	# uses methods from gps_converter to convert list of x,y,z to lat-longs
	def convertMission(self,home, wp_list):

		gps_list = []

		for wp in wp_list:

			lat,lon = gps.xy_to_gps(home.latitude, home.longitude, wp.x, wp.y)[0],gps.xy_to_gps(home.latitude, home.longitude, wp.x, wp.y)[1]
			coord = Coord(lat,lon, wp.z)

			gps_list.append(coord)

		return gps_list		

	def clearCurrentMission(self):

		try:
			clearMissionSrv = rospy.ServiceProxy('/mavros/mission/clear', WaypointClear)
			clearResponse = clearMissionSrv()
			rospy.loginfo(clearResponse)		
		except rospy.ServiceException, e:
			rospy.loginfo('Service call failed: %s' %e)

	def createWaypointPush(self,gps_list):

		self.clearCurrentMission()
		waypointList = WaypointList()
		# waypointList.start_index = 

		for coord in gps_list:

			wp = Waypoint()
			wp.frame = 3
			wp.command = 16 #simple point
			wp.is_current = False
			wp.autocontinue = True
			wp.param1 = 0 #takeoff altitude
			wp.param2 = 0
			wp.param3 = 0
			wp.param4 = 0

			wp.x_lat = coord.latitude
			wp.y_long = coord.longitude
			wp.z_alt = coord.altitude

			waypointList.waypoints.append(wp)

		for wp in waypointList.waypoints:
			print(wp.x_lat, wp.y_long, wp.z_alt)	

		try:
			pushMissionSrv = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
			pushResponse = pushMissionSrv(0,waypointList.waypoints)
			rospy.loginfo(pushResponse)		
		except rospy.ServiceException, e:
			rospy.loginfo('Service call failed: %s' %e)	

	def execute(self, userdata):
	
		wp1 = WP(10,10,10)
		wp2 = WP(20,20,10)
		wp3 = WP(30,30,10)
		wp4 = WP(40,40,10)
		wp5 = WP(50,50,10)

		home = Coord(-35.36326,149.16524)

		wp_list = [wp1,wp2,wp3,wp4,wp5]

		self.createWaypointPush(self.convertMission(home, wp_list))


		try:	#service call to set mode to takeoff
			setModeSrv = rospy.ServiceProxy("/mavros/set_mode", SetMode) #http://wiki.ros.org/mavros/CustomModes
			setModeResponse = setModeSrv(0, 'AUTO')
			rospy.loginfo(TextColors.OKGREEN + str(setModeResponse) + TextColors.ENDC)

		except rospy.ServiceException, e:
			rospy.loginfo(TextColors.FAIL + 'Service call failed: %s' %e + TextColors.ENDC)

		return 'exit_flight'


def main():
	
	pass
	
if __name__ == '__main__':
    main()


