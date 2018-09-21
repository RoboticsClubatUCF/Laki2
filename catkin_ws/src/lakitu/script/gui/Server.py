import interop

class Server:
	
	# client interacts with the interoperability system to get mission data
	client = interop.Client("http://localhost:8000", "testuser", "testpass", 10, 10)
	missions = client.get_missions()
	air_drop_pos = missions[0].air_drop_pos
	home_pos = missions[0].home_pos
	mission_waypoints = missions[0].mission_waypoints
	off_axis_odlc_pos = missions[0].off_axis_odlc_pos
	emergent_last_known_pos = missions[0].emergent_last_known_pos
	search_grid_points = missions[0].search_grid_points
	fly_zones = missions[0].fly_zones


	def get_missions(this):
		return Server.missions

	def add_mission_waypoint(this, gpsWaypoint, altitude):
		if(altitude == 0):
			newWaypoint = interop.Waypoint(len(Server.mission_waypoints)+1, gpsWaypoint[0], gpsWaypoint[1], None)
		else:
			newWaypoint = interop.Waypoint(len(Server.mission_waypoints)+1, gpsWaypoint[0], gpsWaypoint[1], altitude)


		Server.mission_waypoints.append(newWaypoint)
		return Server.mission_waypoints

	def change_air_drop_pos(this, gpsWaypoint):
		Server.air_drop_pos = interop.Waypoint(gpsWaypoint[0], gpsWaypoint[1], None)

	