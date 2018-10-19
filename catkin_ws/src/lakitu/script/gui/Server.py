import interop
import rospy
from sensor_msgs.msg import NavSatFix

# This class handles all of the information being received from the competition server
# It also handles sending and receiving information from ROS
# Received information:
# GPS coordinates, velocity, current waypoint path
# Sent information:
# Mission waypoints, fly zone boundaries, air drop GPS position, emergent last known position (GPS), and search grid waypoints

class Server:
	
	# client interacts with the interoperability system to get mission data
	client = interop.Client("http://localhost:8000", "testuser", "testpass", 10, 10)
	missions = client.get_missions()
	obstacles = client.get_obstacles()

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

	def add_mission_waypoint(this, gpsWaypoint, altitude):
		if(altitude == 0):
			newWaypoint = interop.Waypoint(len(Server.mission_waypoints)+1, gpsWaypoint[0], gpsWaypoint[1], None)
		else:
			newWaypoint = interop.Waypoint(len(Server.mission_waypoints)+1, gpsWaypoint[0], gpsWaypoint[1], altitude)

	
		Server.mission_waypoints.append(newWaypoint)
		
		return Server.mission_waypoints

	def clear_missionWaypoints(this):
		Server.mission_waypoints = []

	def clear_flyZones(this):
		Server.fly_zones = []

	def clear_searchGrid(this):
		Server.search_grid_points = []

	def deleteAirDropPosition(this):
		Server.air_drop_pos = None

	def deleteOdlcPosition(this):
		Server.off_axis_odlc_pos = None

	def deleteEmergent(this):
		Server.emergent_last_known_pos = None

	def add_flyzone_waypoint(this, gpsWaypoint):
		newWaypoint = interop.Waypoint(len(Server.mission_waypoints)+1, gpsWaypoint[0], gpsWaypoint[1], None)
		Server.fly_zones.append(newWaypoint)

		return Server.fly_zones

	def add_searchGridWaypoint(this, gpsWaypoint):
		newWaypoint = interop.Waypoint(len(Server.mission_waypoints)+1, gpsWaypoint[0], gpsWaypoint[1], None)
		Server.search_grid_points.append(newWaypoint)

		return Server.search_grid_points

	def change_air_drop_pos(this, gpsWaypoint):
		Server.air_drop_pos = interop.Waypoint(0,gpsWaypoint[0], gpsWaypoint[1], None)

	def changeEmergent(this, gpsWaypoint):
		newWaypoint = interop.Waypoint(0, gpsWaypoint[0], gpsWaypoint[1], None)
		Server.emergent_last_known_pos = newWaypoint

	def changeOdlcPosition(this, gpsWaypoint):
		newWaypoint = interop.Waypoint(0, gpsWaypoint[0], gpsWaypoint[1], None)
		Server.off_axis_odlc_pos = newWaypoint

	def getObstacles(this):
		return Server.obstacles

	def addObstacle(this, gpsWaypoint, radius, height):
		newObstacle = interop.StationaryObstacle(gpsWaypoint[0], gpsWaypoint[1], radius, height)

	# The below methods handle interacting with ROS

	def callback_CurrentDroneGpsLocation(this, data):
		Server.currentDroneGps.latitude = data.latitude
		Server.currentDroneGps.longitude = data.longitude
		Server.currentDroneGps.altitude_msl = data.altitude

	# Gets the current GPS position of the drone from ROS
	def initializeROSNodes(this):
		rospy.init_node("Server", anonymous=True)
		rospy.Subscriber("/mavros/global_position/global", NavSatFix, this.callback_CurrentDroneGpsLocation)
		# rospy.Subscriber("/mavros/local/_position/velocity", )
	def getCurrentDroneGps(this):
		return Server.currentDroneGps

	