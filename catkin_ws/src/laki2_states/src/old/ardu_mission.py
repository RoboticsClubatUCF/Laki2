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


# DEPRECATED BY FLIGHT, ALL MISSION FUNCTIONALITY IS (HOPEFULLY) IN THE FLIGHT STATE ALREADY

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

# uses methods from gps_converter to convert list of x,y,z to lat-longs
def convertMission(home, wp_list):

	gps_list = []

	for wp in wp_list:

		lat,lon = gps.xy_to_gps(home.latitude, home.longitude, wp.x, wp.y)[0],gps.xy_to_gps(home.latitude, home.longitude, wp.x, wp.y)[1]
		coord = Coord(lat,lon, wp.z)

		gps_list.append(coord)

	return gps_list	

def clearCurrentMission():

	try:
		clearMissionSrv = rospy.ServiceProxy('/mavros/mission/clear', WaypointClear)
		clearResponse = clearMissionSrv()
		rospy.loginfo(clearResponse)		
	except rospy.ServiceException, e:
		rospy.loginfo('Service call failed: %s' %e)

def createWaypointPush(gps_list):

	clearCurrentMission()
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
			


def main():
	
	rospy.init_node('mission_pusher', anonymous=True)

	# rospy.Subscriber("/laki2/flight_target/global", NavSatFix, coordToQueue) #tells where we're going

	wp1 = WP(10,10,10)
	wp2 = WP(20,20,10)
	wp3 = WP(30,30,10)
	wp4 = WP(40,40,10)
	wp5 = WP(50,50,10)

	home = Coord(-35.36326,149.16524)

	wp_list = [wp1,wp2,wp3,wp4,wp5]

	createWaypointPush(convertMission(home, wp_list))



	
if __name__ == '__main__':
    main()



