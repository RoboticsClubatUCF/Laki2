# Made by Matthew Kurtz

# This class handles all of the information being received from the competition server
# It also handles sending and receiving information from ROS
# Received information:
# GPS coordinates, velocity, current waypoint path
# Sent information:
# Mission waypoints, fly zone boundaries, air drop GPS position, emergent last known position (GPS), and search grid waypoints

# TODO: Import the auvsi client interop class (See the SUAS getting started page on github)
# TODO: Test ROS integration

import auvsi_suas as interop
from auvsi_suas.client import Client
import rospy
from sensor_msgs.msg import NavSatFix

class Server:
	
	# client interacts with the interoperability system to get mission data
	s_client = Client.Client("http://localhost:8000", "testuser", "testpass")
	missions = s_client.get_missions()
	obstacles = s_client.get_obstacles()

	air_drop_pos = missions[0].air_drop_pos
	home_pos = missions[0].home_pos
	mission_waypoints = missions[0].mission_waypoints
	off_axis_odlc_pos = missions[0].off_axis_odlc_pos
	emergent_last_known_pos = missions[0].emergent_last_known_pos
	search_grid_points = missions[0].search_grid_points
	fly_zones = missions[0].fly_zones[0].boundary_pts


	currentDroneGps = interop.Waypoint(0, -1, -1, 0)

	print obstacles[0]

	# The below methods handle altering the class variables 
	# That relate to the information we are receiving from the competition server
	def get_missions(this):
		return Server.missions

	# Add waypoints to the waypoint list
	def add_mission_waypoint(this, gpsWaypoint, altitude):
		if(altitude == 0):
			newWaypoint = interop.Waypoint(len(Server.mission_waypoints)+1, gpsWaypoint[0], gpsWaypoint[1], None)
		else:
			newWaypoint = interop.Waypoint(len(Server.mission_waypoints)+1, gpsWaypoint[0], gpsWaypoint[1], altitude)

	
		Server.mission_waypoints.append(newWaypoint)
		
		return Server.mission_waypoints

	# Clears the waypoint list
	def clear_missionWaypoints(this):
		Server.mission_waypoints = []

	# Clears the flyzone list
	def clear_flyZones(this):
		Server.fly_zones = []

	# Clears the search grid list of points
	def clear_searchGrid(this):
		Server.search_grid_points = []

	# Resets the Air Drop Position
	def deleteAirDropPosition(this):
		Server.air_drop_pos = None

	# Resets the Odlc Position
	def deleteOdlcPosition(this):
		Server.off_axis_odlc_pos = None

	# Resets the Emergent waypoint
	def deleteEmergent(this):
		Server.emergent_last_known_pos = None

	# Adds a point to the flyzone list
	def add_flyzone_waypoint(this, gpsWaypoint):
		newWaypoint = interop.Waypoint(len(Server.mission_waypoints)+1, gpsWaypoint[0], gpsWaypoint[1], None)
		Server.fly_zones.append(newWaypoint)

		return Server.fly_zones

	# Adds a point to the search grid waypoint list
	def add_searchGridWaypoint(this, gpsWaypoint):
		newWaypoint = interop.Waypoint(len(Server.mission_waypoints)+1, gpsWaypoint[0], gpsWaypoint[1], None)
		Server.search_grid_points.append(newWaypoint)

		return Server.search_grid_points

	# Changes the air drop position waypoint to another waypoint
	def change_air_drop_pos(this, gpsWaypoint):
		Server.air_drop_pos = interop.Waypoint(0,gpsWaypoint[0], gpsWaypoint[1], None)

	# Changes Emergent waypoint to another waypoint
	def changeEmergent(this, gpsWaypoint):
		newWaypoint = interop.Waypoint(0, gpsWaypoint[0], gpsWaypoint[1], None)
		Server.emergent_last_known_pos = newWaypoint

	# Changes Odlc waypoint to another waypoint
	def changeOdlcPosition(this, gpsWaypoint):
		newWaypoint = interop.Waypoint(0, gpsWaypoint[0], gpsWaypoint[1], None)
		Server.off_axis_odlc_pos = newWaypoint

	# Returns the list of obstacles
	def getObstacles(this):
		return Server.obstacles

	# Add a obstacle to the obstacle list
	def addObstacle(this, gpsWaypoint, radius, height):
		newObstacle = interop.StationaryObstacle(gpsWaypoint[0], gpsWaypoint[1], radius, height)

#----------------------------------------------------------------------------------------------------------#
	# The below methods handle interacting with ROS:

	# Updates Drone GPS Location from ROS
	def callback_CurrentDroneGpsLocation(this, data):
		Server.currentDroneGps.latitude = data.latitude
		Server.currentDroneGps.longitude = data.longitude
		Server.currentDroneGps.altitude_msl = data.altitude

	# Initialize ROS
	def initializeROSNodes(this):
		rospy.init_node("Server", anonymous=True)
		rospy.Subscriber("/mavros/global_position/global", NavSatFix, this.callback_CurrentDroneGpsLocation)
		# rospy.Subscriber("/mavros/local/_position/velocity", )

	# Returns the current GPS position of the drone
	def getCurrentDroneGps(this):
		return Server.currentDroneGps

	