#!/usr/bin/env python

import rospy, mavros
import math, smach, smach_ros
from mavros_msgs.srv import CommandBool, SetMode, ParamGet, WaypointPush, WaypointClear
from mavros_msgs.msg import State, RCIn, Waypoint, WaypointList
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix

import laki2_common.gps_converter as laki2_GPS
from laki2_common import TextColors
from laki2_msg.msg import MissionPath

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

# uses ROS service to set ARDUPILOT mode
# takes string with mode name, ALL CAPS
# WORKING, TESTED: AUTO, LAND
def setMode(mode):

	try:	#service call to set mode to auto
		setModeSrv = rospy.ServiceProxy("/mavros/set_mode", SetMode) #http://wiki.ros.org/mavros/CustomModes
		setModeResponse = setModeSrv(0, mode)
		rospy.loginfo(TextColors.OKGREEN + str(setModeResponse) + TextColors.ENDC)
	except rospy.ServiceException, e:
		rospy.loginfo(TextColors.FAIL + 'Service call failed: %s' %e + TextColors.ENDC)

	return setModeResponse	    


# SM State: MISSION
# From: 	TAKEOFF
# To:		EXIT FLIGHT_SM
# Purpose: 	main flight state; builds the mission that is passed to ArduPilot, then waits for more missions
class Mission(smach.State):
	
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
			lat,lon = lat_long[0], lat_long[1]
			coord = Coord(lat,lon, wp.z)

			gps_list.append(coord)

		return gps_list		

	def createMission(self,gps_list):

		waypointList = WaypointList()

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

		self.mission_ready = True		
		return waypointList	
		

	def pushMission(self, mission):

		try:
			pushMissionSrv = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
			pushResponse = pushMissionSrv(0,mission.waypoints)
			rospy.loginfo('FUCK YOURSELF')
			rospy.loginfo(pushResponse)		
		except rospy.ServiceException, e:
			rospy.loginfo('Service call failed: %s' %e)	

		self.mission_ready = None	

	def getMissionWPs(self, data):

		self.clearCurrentMission()	

		self.wp_list = []
		
		for point in data.points:
			new_wp = WP(point.x, point.y, point.z)
			self.wp_list.append(new_wp)
			
		self.mission_ready = False	

	def getState(self, data):

		self.state = data
			
	def __init__(self):
		smach.State.__init__(self,outcomes=['toLAND','exit_flight'])
		self.wp_list = []
		self.mission_ready = None
		self.state = None
		rospy.Subscriber("/laki2/mission/waypoints", MissionPath, self.getMissionWPs)
		rospy.Subscriber("/mavros/state", State, self.getState)

	def execute(self, userdata):
	
		self.clearCurrentMission()

		rate = rospy.Rate(30)

		while not rospy.is_shutdown():

			rate.sleep()

			# hard-coded start point, ideally this will be done dynamically in the future
			home = Coord(-35.36326,149.16524)

			if self.mission_ready is None:
				rospy.loginfo(self.mission_ready)
				continue
			elif not self.mission_ready:
				rospy.loginfo(self.mission_ready)
				gps_list = self.convertToGPS(home, self.wp_list)
				mission = self.createMission(gps_list)
				# self.createMission(self.convertToGPS(home, wp_list))
			else:
				rospy.loginfo(self.mission_ready)
				self.pushMission(mission)
				if self.state.mode is not 'AUTO':
					setMode('AUTO')

		return 'exit_flight'


