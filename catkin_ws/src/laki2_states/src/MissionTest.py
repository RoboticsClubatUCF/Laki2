#!/usr/bin/env python

import rospy, mavros
import math, smach, smach_ros
from mavros_msgs.srv import CommandBool, SetMode, ParamGet, CommandTOL
from mavros_msgs.msg import State, RCIn
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped

from laki2_common import *
import laki2_common.gps_converter as laki2_GPS
from laki2_msg.msg import MissionPath, FlightStatus, Coordinate, MissionCoords

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

# uses methods from gps_converter to convert list of (x,y,z)s (WPs) to lat-longs (Coords)
def convertToGPS(home, wp_list):

	gps_list = []

	for wp in wp_list:

		lat_long = laki2_GPS.xy_to_gps(home.latitude, home.longitude, wp.x, wp.y)
		# lat,lon = laki2_GPS.xy_to_gps(home.latitude, home.longitude, wp.x, wp.y)[0],laki2_GPS.xy_to_gps(home.latitude, home.longitude, wp.x, wp.y)[1]
		lat,lon = lat_long[0], lat_long[1]
		coord = Coord(lat,lon, wp.z)

		gps_list.append(coord)

	return gps_list		


def buildMessage(gps_list):

	mission = MissionCoords()
	coord = Coordinate()

	for gps in gps_list:

		coord.latitude = gps.latitude
		coord.longitude = gps.longitude
		coord.altitude = gps.altitude

		mission.coords.append(coord)


	# mission.coords = gps_list

	return mission	


def MissionTest():

	# a hardcoded mission for testing
	wp1 = WP(10,10,10)
	wp2 = WP(20,20,10)
	wp3 = WP(30,30,10)
	wp4 = WP(40,40,10)
	wp5 = WP(50,50,10)

	# hard-coded start point, ideally this will be done dynamically in the future
	home = Coord(-35.36326,149.16524)

	wp_list = [wp1,wp2,wp3,wp4,wp5]

	gps_list = convertToGPS(home, wp_list)

	mission = buildMessage(gps_list)

	mission_pub = rospy.Publisher('/laki2/mission', MissionCoords, queue_size=10)

	while not rospy.is_shutdown():

		mission_pub.publish(mission)



def main():

	rospy.init_node('mission_test', anonymous=True)

	# mission_pub = rospy.Publisher('/laki2/mission', MissionPath, queue_size=10)

	MissionTest()


if __name__ == '__main__':
    main()
