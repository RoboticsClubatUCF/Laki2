#!/usr/bin/env python

import rospy, mavros
import math, smach, smach_ros
from mavros_msgs.srv import CommandBool, SetMode, ParamGet, WaypointPush, WaypointClear
from mavros_msgs.msg import State, RCIn, Waypoint, WaypointList
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix

import laki2_common.gps_converter as laki2_GPS

# simple Waypoint (WP) class to hold (x,y,z)s 
# used in MISSION state to build the mission
class WP:

	def __init__(self, x, y, z):
		self.x = x
		self.y = y
		self.z = z

# simple GPS coordinate class
# coords are used as mission points in MISSION state
class Coord:

	def __init__(self, latitude, longitude, altitude=0):
		self.latitude = latitude
		self.longitude = longitude
		self.altitude = altitude

# # totally unnecessary class to make terminal output pretty
# class TextColors:
#     HEADER = '\033[95m'
#     OKBLUE = '\033[94m'
#     OKGREEN = '\033[92m'
#     WARNING = '\033[93m'
#     FAIL = '\033[91m'
#     ENDC = '\033[0m'
#     BOLD = '\033[1m'
#     UNDERLINE = '\033[4m'		

# uses ROS service to set ARDUPILOT mode
# takes string with mode name, ALL CAPS
# made because this method is used multiple times throughout
def setMode(mode):

	try:	#service call to set mode to auto
		setModeSrv = rospy.ServiceProxy("/mavros/set_mode", SetMode) #http://wiki.ros.org/mavros/CustomModes
		setModeResponse = setModeSrv(0, mode)
		rospy.loginfo(TextColors.OKGREEN + str(setModeResponse) + TextColors.ENDC)
	except rospy.ServiceException, e:
		rospy.loginfo(TextColors.FAIL + 'Service call failed: %s' %e + TextColors.ENDC)

	return setModeResponse	    

# # just holds the copter in Standby state, doesn't do anything
# class Standby(smach.State):
	
# 	def __init__(self):
# 		smach.State.__init__(self, outcomes=['toMISSION','exit_flight'])

# 	def execute(self, userdata):
	
# 		while not rospy.is_shutdown():

# 			return 'toMISSION'

# 			rospy.spin()	



# SM State: MISSION
# From: 	TAKEOFF
# To:		EXIT FLIGHT_SM
# Purpose: 	main flight state; builds the mission that is passed to ArduPilot, then waits for more missions
class Mission(smach.State):
	
	def __init__(self):
		smach.State.__init__(self,outcomes=['toLAND','exit_flight'])

	# clears missions from ArduPilot	
	def clearCurrentMission(self):

		try:
			clearMissionSrv = rospy.ServiceProxy('/mavros/mission/clear', WaypointClear)
			clearResponse = clearMissionSrv()
			rospy.loginfo(clearResponse)		
		except rospy.ServiceException, e:
			rospy.loginfo('Service call failed: %s' %e)
	

	# uses methods from gps_converter to convert list of (x,y,z)s (WPs) to lat-longs (Coords)
	def convertToGPS(self, home, wp_list):

		gps_list = []

		for wp in wp_list:

			lat_long = laki2_GPS.xy_to_gps(home.latitude, home.longitude, wp.x, wp.y)
			# lat,lon = laki2_GPS.xy_to_gps(home.latitude, home.longitude, wp.x, wp.y)[0],laki2_GPS.xy_to_gps(home.latitude, home.longitude, wp.x, wp.y)[1]
			lat,lon = lat_long[0], lat_long[1]
			coord = Coord(lat,lon, wp.z)

			gps_list.append(coord)

		return gps_list		

	
	def createMission(self,gps_list):

		waypointList = WaypointList()
		# waypointList.start_index = 
		
		for coord in gps_list:

			wp = Waypoint()
			wp.frame = 3
			wp.command = 16 #simple point, full list in doc/MissionMessage.txt
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
	
		self.clearCurrentMission()

		# a hardcoded mission for testing
		wp1 = WP(10,10,10)
		wp2 = WP(20,20,10)
		wp3 = WP(30,30,10)
		wp4 = WP(40,40,10)
		wp5 = WP(50,50,10)

		# hard-coded start point, ideally this will be done dynamically in the future
		home = Coord(-35.36326,149.16524)

		wp_list = [wp1,wp2,wp3,wp4,wp5]

		gps_list = self.convertToGPS(home, wp_list)
		self.createMission(gps_list)
		# self.createMission(self.convertToGPS(home, wp_list))


		try:	#service call to set mode to auto
			setModeSrv = rospy.ServiceProxy("/mavros/set_mode", SetMode) #http://wiki.ros.org/mavros/CustomModes
			setModeResponse = setModeSrv(0, 'AUTO')
			rospy.loginfo(TextColors.OKGREEN + str(setModeResponse) + TextColors.ENDC)
		except rospy.ServiceException, e:
			rospy.loginfo(TextColors.FAIL + 'Service call failed: %s' %e + TextColors.ENDC)

		while not rospy.is_shutdown():
		
			rospy.spin()	


		return 'exit_flight'


