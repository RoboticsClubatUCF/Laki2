from tkinter import *
import cv2, numpy as np, mavros, rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
# for getGpsImage:
import Image, urllib, StringIO

# variables updated by ROS (initialized here to ensure global access)
rosGpsData = None
latitude = None
longitude = None
altitude = None
current_pos = None
root = None


circleList = []

# global variables needed for gui objects
label_gpsData = None
label_imuData = None
label_altitudeData = None
obstacle_height_input = None
obstacle_radius_input = None

# relates to the image the gui displays of the GPS area
pixel_height = None
pixel_width = None

# containers for all the gui objects that get added
search_grid_circles = []
search_grid_lines = []
flyzone_circles = []
flyzone_lines = []
mission_waypoints_circles = []
mission_waypoints_lines = []
emergent_LKS_circle = -1
odlcPosition_circle = -1
airDrop_circle = -1
obstacles = []
obstacles_circles = []



canvas = None

buttonCommand = "no_command"

# gets the odometry data from ROS and saves it to the variable current_pos
def getCurrentPosition(data):

	global current_pos
	current_pos = data
	
# gets the GPS data from ROS and saves the raw data to rosGpsData, 
# and seperates the latitude and longitude into variables with their respective names
def GetGpsCoords_helper(data):
	global rosGpsData
	global latitude
	global longitude
	
	rosGpsData = data
	latitude = data.latitude
	longitude = data.longitude


# initializes all needed ROS nodes
def init_ros():
	rospy.init_node('gui_gpsListener', anonymous=True)
	rospy.Subscriber("/mavros/global_position/global", NavSatFix, GetGpsCoords_helper)
	rospy.Subscriber('/mavros/local_position/odom', Odometry, getCurrentPosition)
	
	
# update the gui with new info
def update():
	global canvas
	global label_altitudeData
	global label_gpsData
	currentDroneGps = Server.getCurrentDroneGps()

	gpsString = str(currentDroneGps.latitude) + ", " + str(currentDroneGps.longitude)
	altitudeString = str(currentDroneGps.altitude_msl)
	# #print altitudeString

	if(label_gpsData != None and label_altitudeData != None):
		label_gpsData.config(text=gpsString)
		label_altitudeData.config(text=altitudeString)
	# label_imuData.config(text=current_pos)


	# change the click listener to what we have selected with the buttons
	canvas.bind("<Button-1>", clickListener) 

	root.after(100, update)



# THIS METHOD IS A TEST OF UPDATING THE GUI, USE THIS TEMPLATE AS NEEDED
# updates the label after a certain amount of time to counter 
# counter = 0
# def my_after(): 
# 	global counter
# 	counter = counter + 1	
	# #print counter
# 	new_text = counter 
	
# 	#change text of label
# 	label_gpsData.config(text=new_text)
	
# 	# call again after 100 ms
# 	root.after(100, my_after)
# you must call the method at least once to get it started, it will continue on its own afterwards
#my_after()
	
# creates a red line between given gps coordinates
# TKinter canvas defines pixels with origin on the top-left:
#	   (0,0)-----------------(X_max, 0)
#	        |
#		    |
#		    |
# (0, Y_max)|
# TO USE WITH gpsToPixel YOU MAKE THE Y COORDINATES NEGATIVE TO FIT WITH TKINTER CANVAS PIXELS (which are 4th quadrant)
def createLine(gps1, gps2, color):
	global pixel_width	# correlates to longitude
	global pixel_height # correlates to latitude

	corners = getCorners(0)

	x1 = gpsToPixel(gps1[1], "lon") 
	x2 = gpsToPixel(gps2[1], "lon")
	y2 = gpsToPixel(gps1[0], "lat")
	y1 = gpsToPixel(gps2[0], "lat")


	# #print "\nmaking line from (gps) = " + str(gps1) + " to " + str(gps2) + ")"
	# #print "making line from (pixel) = (" + str(x1) + ", " + str(y1) + ") to (" + str(x2) + ", " + str(y2) + ")"

	return canvas.create_line(x1,y2,x2,y1, fill=color)

# converts gps coords to pixel size using the equation:
# (b/(B-A)(X-A)) = p
# where B is the far corner, A is the initial corner, b is the image size in pixels,
# and X is the gps coordinate to be converted
# ex)
# 	A--------------B
#    |     x      |
#    |            |
#	 -------------- b
# paramaters: 
# 		gps: the desired gps coordinate to convert
#		imageSize: how long the side is of the  image that you are converting to.
#		A: the lower bound of the GPS coordinate
#		B: the upper bound of the GPS coordinate
def gpsToPixel(gps,latOrLon):
	# #print str((gps)) + " " + str((imageSize)) + " " + str((A)) + " " +  str((B))
	global pixel_width	# correlates to longitude
	global pixel_height # correlates to latitude

	corners = getCorners(0)

	Alon = corners[0][1]
	Blon = corners[1][1]
	
	Alat = corners[0][0]
	Blat = corners[2][0]
	

	if(latOrLon == "lat"):
		result = (pixel_height / (Blat-Alat)) * (gps-Alat)

	elif(latOrLon == "lon"):
		result = (pixel_width / (Blon-Alon))*(gps-Alon)

	if (result == -0.0):
		return 0
	else:
		return int(result)
	

def pixelToGps(pixel, xOrY):
	global pixel_width
	global pixel_height

	corners = getCorners(0)

	Alon = corners[0][1]
	Blon = corners[1][1]
	
	Alat = corners[0][0]
	Blat = corners[2][0]

	if(xOrY == "y"):
		result = (((Blat-Alat)/pixel_height)*pixel[0]) + Alat
	elif(xOrY == "x"):
		result = (((Blon-Alon)/pixel_width)*pixel[1]) + Alon
	return result

# returns GPS coords for each corner in the flight zone as given by the Server
# in the indexes following the convention:
#  0-----------1
#  |           |
#  |           |
#  2-----------3
# parameters: if passed 1, return as string, otherwise return as a float
def getCorners(stringReturn):
	global Server
	# gets the missions from the Server
	missions = Server.get_missions()

	# boundary points for the first mission, and first fly zone
	boundary_pts = missions[0].fly_zones[0].boundary_pts

	latitude = []
	longitude = []

	for i in range(0,len(boundary_pts)):
		waypoint = boundary_pts[i]

		latitude.append(waypoint.latitude)
		longitude.append(waypoint.longitude)

	latitude_min = min(latitude)
	latitude_max = max(latitude)

	longitude_min = min(longitude)
	longitude_max = max(longitude)

	top_left = (latitude_max, longitude_min)
	top_right = (latitude_max, longitude_max)
	bottom_left = (latitude_min, longitude_min)
	bottom_right = (latitude_min, longitude_max)

	str_top_left = str(top_left[0]) + ", " + str(top_left[1])
	str_top_right = str(top_right[0]) + ", " + str(top_right[1])
	str_bot_left = str(bottom_left[0]) + ", " + str(bottom_left[1])
	str_bot_right = str(bottom_right[0]) + ", " + str(bottom_right[1])

	if (stringReturn is 1):
		return (str_top_left, str_top_right, str_bot_left, str_bot_right)
	else:
		return (top_left, top_right, bottom_left, bottom_right)

def removeLastCircle():
	global canvas
	global circleList
	# #print circleList
	if(len(circleList) > 0):
		canvas.delete(circleList[(len(circleList)-1)])
		circleList.pop()

# gets the gps image from google with specified latitude and longitude coords
def getGpsImage(upperLeftCoords, lowerRightCoords):
	EARTH_RADIUS = 6378137
	EQUATOR_CIRCUMFERENCE = 2 * pi * EARTH_RADIUS
	INITIAL_RESOLUTION = EQUATOR_CIRCUMFERENCE / 256.0
	ORIGIN_SHIFT = EQUATOR_CIRCUMFERENCE / 2.0

	def latlontopixels(lat, lon, zoom):
		mx = (lon * ORIGIN_SHIFT) / 180.0
		my = log(tan((90 + lat) * pi/360.0))/(pi/180.0)
		my = (my * ORIGIN_SHIFT) /180.0
		res = INITIAL_RESOLUTION / (2**zoom)
		px = (mx + ORIGIN_SHIFT) / res
		py = (my + ORIGIN_SHIFT) / res
		return px, py

	def pixelstolatlon(px, py, zoom):
		res = INITIAL_RESOLUTION / (2**zoom)
		mx = px * res - ORIGIN_SHIFT
		my = py * res - ORIGIN_SHIFT
		lat = (my / ORIGIN_SHIFT) * 180.0
		lat = 180 / pi * (2*atan(exp(lat*pi/180.0)) - pi/2.0)
		lon = (mx / ORIGIN_SHIFT) * 180.0
		return lat, lon

	############################################

	# Put GPS coordinates here to get satellite image from google
	upperleft = upperLeftCoords
	lowerright= lowerRightCoords

	zoom = 18   # be careful not to get too many images!

	############################################

	ullat, ullon = map(float, upperleft.split(','))
	lrlat, lrlon = map(float, lowerright.split(','))

	# Set some important parameters
	scale = 1
	maxsize = 640

	# convert all these coordinates to pixels
	ulx, uly = latlontopixels(ullat, ullon, zoom)
	lrx, lry = latlontopixels(lrlat, lrlon, zoom)

	# calculate total pixel dimensions of final image
	dx, dy = lrx - ulx, uly - lry

	# calculate rows and columns
	cols, rows = int(ceil(dx/maxsize)), int(ceil(dy/maxsize))

	# calculate pixel dimensions of each small image
	bottom = 120
	largura = int(ceil(dx/cols))
	altura = int(ceil(dy/rows))
	alturaplus = altura + bottom


	final = Image.new("RGB", (int(dx), int(dy)))
	for x in range(cols):
		for y in range(rows):
		    dxn = largura * (0.5 + x)
		    dyn = altura * (0.5 + y)
		    latn, lonn = pixelstolatlon(ulx + dxn, uly - dyn - bottom/2, zoom)
		    position = ','.join((str(latn), str(lonn)))
		    #print x, y, position
		    urlparams = urllib.urlencode({'center': position,
		                                  'zoom': str(zoom),
		                                  'size': '%dx%d' % (largura, alturaplus),
		                                  'maptype': 'satellite',
		                                  'sensor': 'false',
		                                  'scale': scale})
		    url = 'http://maps.google.com/maps/api/staticmap?' + urlparams
		    f=urllib.urlopen(url)
		    im=Image.open(StringIO.StringIO(f.read()))
		    final.paste(im, (int(x*largura), int(y*altura)))
	
	final.save("gps.png", "PNG")
	#final.show()

# makes a circle in the Tkinter canvas around a point (centroid) with a given radius
# note: centroid is a tuple of gps coordinates: (latitude, longitude)
def makeCircle(centroid, radius, color):
	if(radius == 0):
		radius = 20
	lat = gpsToPixel(centroid[0], "lat")
	lon = gpsToPixel(centroid[1], "lon")

	TL = [lon-radius, lat+radius] # top left coordinates
	BR = [lon+radius, lat-radius] # top right coordinates

	#id = C.create_oval(x0, y0, x1, y1, option, ...)
	global canvas


	return canvas.create_oval(TL[0], TL[1], BR[0], BR[1], fill=color)

def clickCallback_noCommand(event):
	pixel = (event.y, event.x)

	lon = pixelToGps(pixel, "x")
	lat = pixelToGps(pixel, "y")

	#print "No command (clicked at (lat: " + str(lat) + ", lon:" + str(lon) + "))"

def clickCallback_addFlyzoneWaypoints(event):
	global Server
	global flyzone_lines
	
	pixel = (event.y, event.x)
	#print "adding flyzone waypoint"
	lon = pixelToGps(pixel, "x")
	lat = pixelToGps(pixel, "y")

	waypoints = Server.fly_zones
	Server.add_flyzone_waypoint((lat,lon))
	if(len(Server.fly_zones) >=2):
		if(len(flyzone_lines) > 0):
			canvas.delete(flyzone_lines.pop())
		flyzone_lines.append(createLine((lat, lon), (waypoints[len(waypoints)-2].latitude, waypoints[len(waypoints)-2].longitude), "red"))
		flyzone_lines.append(createLine((lat, lon), (waypoints[0].latitude, waypoints[0].longitude), "red"))

# TODO: MAKE THIS WORK
def clickCallback_addSearchGridWaypoints(event):
	global search_grid_lines
	if event != None:
		pixel = (event.y, event.x)
		#print "adding search grid waypoint"
		lon = pixelToGps(pixel, "x")
		lat = pixelToGps(pixel, "y")

		waypoints = Server.search_grid_points
		Server.add_searchGridWaypoint((lat, lon))
		if(len(Server.search_grid_points) >= 2):
			if(len(search_grid_lines) > 0):
				canvas.delete(search_grid_lines.pop())
		search_grid_lines.append(createLine((lat, lon), (waypoints[len(waypoints)-2].latitude, waypoints[len(waypoints)-2].longitude), "orange"))
		search_grid_lines.append(createLine((lat, lon), (waypoints[0].latitude, waypoints[0].longitude), "orange"))

def clickCallback_addMissionWaypoints(event):
	global mission_waypoints_circles
	global mission_waypoints_lines

	pixel = (event.y, event.x)
	#print "adding misison waypoint"
	lon = pixelToGps(pixel, "x")
	lat = pixelToGps(pixel, "y")

	waypoints = Server.mission_waypoints
	Server.add_mission_waypoint((lat, lon), None)
	if(len(Server.mission_waypoints) >=2):
		if(len(mission_waypoints_lines) > 0):
			canvas.delete(mission_waypoints_lines.pop())

		mission_waypoints_lines.append(createLine((lat, lon), (waypoints[len(waypoints)-2].latitude, waypoints[len(waypoints)-2].longitude), "blue"))
		mission_waypoints_lines.append(createLine((lat, lon), (waypoints[0].latitude, waypoints[0].longitude), "blue"))

def clickCallback_changeEmergent(event):
	global emergent_LKS_circle
	global canvas
	if event != None:
		pixel = (event.y, event.x)
		#print "changing emergent object"
		lon = pixelToGps(pixel, "x")
		lat = pixelToGps(pixel, "y")

		emergent = Server.emergent_last_known_pos
		Server.changeEmergent((lat,lon))
		if(emergent_LKS_circle != -1):
			canvas.delete(emergent_LKS_circle)
		emergent_LKS_circle = makeCircle((lat, lon), 0, "purple")

def clickCallback_changeOdlcPosition(event):
	global odlcPosition_circle
	if event != None:
		pixel = (event.y, event.x)
		lon = pixelToGps(pixel, "x")
		lat = pixelToGps(pixel, "y")

		#print "changing odlc"
		Server.changeOdlcPosition((lat,lon))
		if(odlcPosition_circle != -1):
			canvas.delete(odlcPosition_circle)
		odlcPosition_circle = makeCircle((lat,lon), 20, "white")

def clickCallback_changeAirdropPosition(event):
	global airDrop_circle
	if odlcPosition_circle != None:
		pixel = (event.y, event.x)
		#print "changing emergent object"
		lon = pixelToGps(pixel, "x")
		lat = pixelToGps(pixel, "y")

		#print "changing odlc"
		Server.change_air_drop_pos((lat,lon))
		canvas.delete(airDrop_circle)
		airDrop_circle = makeCircle((lat,lon), 20, "orange")

def clickCallback_addObstacle(event):
	global obstacles_circles
	global obstacle_height_input
	global obstacle_radius_input

	#TODO: convert radius from meters to pixels
	# radius = gpsToPixel(float(obstacle_radius_input.get()), 'lat')
	radius = 20
	height = float(obstacle_height_input.get())

	if odlcPosition_circle != None:
		pixel = (event.y, event.x)
		lon = pixelToGps(pixel, "x")
		lat = pixelToGps(pixel, "y")

		Server.addObstacle((lat,lon), radius, height)
		obstacles_circles.append(makeCircle((lat,lon), radius, "red"))

def clearMissionWaypoints():
	global clickListener
	global mission_waypoints_lines
	global canvas

	clickListener = clickCallback_noCommand

	for i in mission_waypoints_lines:
		canvas.delete(i)

	mission_waypoints_lines = []
	Server.clear_missionWaypoints()

	#print "clear mission waypoints"

def addFlyzoneWaypoints():
	global clickListener
	clickListener = clickCallback_addFlyzoneWaypoints

def clearFlyzoneWaypoints():
	global clickListener
	global flyzone_waypoints
	global flyzone_lines
	global flyzone_circles
	global canvas

	for i in flyzone_lines:
		canvas.delete(i)

	for i in flyzone_circles:
		canvas.delete(i)

	flyzone_lines = []
	flyzone_circles = []
	Server.clear_flyZones()
	flyzone_waypoints = []

	clickListener = clickCallback_noCommand

def addSearchGridWaypoints():
	global clickListener
	clickListener = clickCallback_addSearchGridWaypoints

def clearSearchGridWaypoints():
	global clickListener
	global search_grid_waypoints
	global search_grid_lines
	global canvas

	for i in search_grid_lines:
		canvas.delete(i)

	search_grid_waypoints = []
	Server.clear_searchGrid()

	clickListener = clickCallback_noCommand

def changeEmergent():
	global clickListener

	#print "change emergent"

	clickListener = clickCallback_changeEmergent

def changeOdlcPosition():
	global clickListener

	#print "change odlc"

	clickListener = clickCallback_changeOdlcPosition

def changeAirDropPosition():
	global clickListener

	#print "change airdrop position"

	clickListener = clickCallback_changeAirdropPosition

def addMissionWaypoints():
	global clickListener

	#print "add mission waypoints"
	clickListener = clickCallback_addMissionWaypoints

def confirmWaypointSelection():
	global clickListener
	clickListener = clickCallback_noCommand
	waypoints = Server.mission_waypoints
	createLine((waypoints[0].latitude, waypoints[0].longitude), (waypoints[len(waypoints)-1].latitude, waypoints[len(waypoints)-1].longitude), "red")

def deleteAirDropPosition():
	global clickListener
	global airDrop_circle
	clickListener = clickCallback_noCommand
	Server.deleteAirDropPosition()
	canvas.delete(airDrop_circle)
	airDrop_circle = -1

def deleteOdlcPosition():
	global clickListener
	global odlcPosition_circle
	clickListener = clickCallback_noCommand
	Server.deleteOdlcPosition()
	canvas.delete(odlcPosition_circle)
	odlcPosition_circle = -1

def deleteEmergent():
	global clickListener
	global emergent_LKS_circle
	clickListener = clickCallback_noCommand
	Server.deleteEmergent()
	canvas.delete(emergent_LKS_circle)
	emergent_LKS_circle = -1

def addObstacle():
	global clickListener
	clickListener = clickCallback_addObstacle

clickListener = clickCallback_noCommand

def main():
	global root
	global label_gpsData
	global label_imuData
	global label_altitudeData
	global latitude
	global longitude
	global pixel_height
	global pixel_width
	global canvas
	global clickListener

	global search_grid_waypoints_circles
	global flyzone_circles 
	global flyzone_lines 
	global mission_waypoints_circles
	global mission_waypoints_lines
	global emergent_LKS_Circle
	global obstacles
	global obstacle_height_input
	global obstacle_radius_input

	# initialize ros nodes
	# init_ros() 

	#initialize a Tkinter object
	root = Tk()
	
	obstacle_height_input = StringVar()
	obstacle_radius_input = StringVar()

	obstacle_height_input.set("New Obstacle Height")
	obstacle_radius_input.set("New Obstacle Radius")
	
	# default gps for simulator:
	# latitude: 47.3977417
	# longitude: 8.5455941
	
	# get image data (gps satellite data)
	path = "ANTIALIAS.png"
	img = PhotoImage(file=path) # used for Tkinter canvas object
	cv_image = cv2.imread(path) # used to get pixel measurements
	pixel_height = np.size(cv_image, 0) # height of image in pixels
	pixel_width = np.size(cv_image, 1)  # width of image in pixels

	# resize image to desired pixel dimensions
	# open an image file (.bmp,.jpg,.png,.gif) you have in the working folder
	imageFile = "gps.png"
	im1 = Image.open(imageFile)
	# adjust width and height to your needs
	desired_width = 1200
	desired_height = 600

	# use one of these filter options to resize the image
	im5 = im1.resize((desired_width, desired_height), Image.ANTIALIAS)    # best down-sizing filter
	ext = ".png"

	im5.save("ANTIALIAS" + ext)

	#display an image from file path
	canvas = Canvas(root, width = pixel_width, height = pixel_height)
	canvas.grid(row=0, column=0,rowspan=20, columnspan=10)
	canvas.create_image(0,0, anchor = NW, image = img)

	first_col = 11
	second_col = 12
	third_col = 13

	rosRows = 4;

	#Label experimenting
	#label that provides info text
	label_gpsName = Label(root, text="GPS coordinates = ")
	label_gpsName.grid(row=0, column=first_col)

	#label that updates based on given gps data
	label_gpsData = Label(root, text="NO GPS DATA")
	label_gpsData.grid(row=0, column=second_col)

	label_altitudeName = Label(root, text="Altitude = ")
	label_altitudeName.grid(row = 1, column = first_col)

	label_altitudeData = Label(root, text = "NO GPS DATA")
	label_altitudeData.grid(row = 1, column = second_col)

	# put the icon to the left of the text label
	b_addMissionWaypoints = Button(text="Add Mission Waypoints", command=addMissionWaypoints, width=20)
	b_addMissionWaypoints.grid(row=rosRows+1, column=first_col)

	b_clearMissionWaypoints = Button(text="Clear Mission Waypoints", command=clearMissionWaypoints, width=20)
	b_clearMissionWaypoints.grid(row=rosRows+1, column=second_col,)

	b_confirmWaypointSelection = Button(text="Finish Adding Waypoints", command=confirmWaypointSelection, width=45)
	b_confirmWaypointSelection.grid(row=rosRows+2, column=first_col, columnspan = 2)

	b_changeAir_drop_pos = Button(text="Change Air Drop Waypoint", command=changeAirDropPosition, width=20)
	b_changeAir_drop_pos.grid(row=rosRows+3, column=first_col)

	b_deleteAir_drop_pos = Button(text="Delete Air Drop Waypoint", command=deleteAirDropPosition, width=20)
	b_deleteAir_drop_pos.grid(row=rosRows+3, column=second_col)

	b_changeOff_axis_odlc_pos = Button(text="Change odlc position", command=changeOdlcPosition, width=20)
	b_changeOff_axis_odlc_pos.grid(row=rosRows+4,column=first_col)

	b_deleteOff_axis_odlc_pos = Button(text="Delete odlc position", command=deleteOdlcPosition, width=20)
	b_deleteOff_axis_odlc_pos.grid(row=rosRows+4,column=second_col)

	b_changeEmergent_last_known_pos = Button(text="Change emergent LKP", command=changeEmergent, width=20)
	b_changeEmergent_last_known_pos.grid(row=rosRows+5, column=first_col)

	b_deleteEmergent_last_known_pos = Button(text="Delete emergent LKP", command=deleteEmergent, width=20)
	b_deleteEmergent_last_known_pos.grid(row=rosRows+5, column=second_col)

	b_addSearch_grid_points = Button(text="Add Search Grid Waypoints", command=addSearchGridWaypoints, width=20)
	b_addSearch_grid_points.grid(row=rosRows+6, column=first_col)

	b_clearSearch_grid_points = Button(text="Clear Search Grid Waypoints", command=clearSearchGridWaypoints, width=20)
	b_clearSearch_grid_points.grid(row=rosRows+6, column=second_col)

	b_addFly_zones_Waypoints = Button(text="Add FlyZone Waypoints", command=addFlyzoneWaypoints, width=20)
	b_addFly_zones_Waypoints.grid(row=rosRows+7, column=first_col)

	b_clearFlyzone_Waypoints = Button(text="Clear FlyZone Waypoints", command=clearFlyzoneWaypoints, width=20)
	b_clearFlyzone_Waypoints.grid(row=rosRows+7, column = second_col)

	b_addObstacle = Button(text="Add Obstacle", command=addObstacle, width=45)
	b_addObstacle.grid(row=rosRows+9, column = first_col, columnspan = 2)

	et_obstacleHeight = Entry(root, textvariable=obstacle_height_input, width=22)
	et_obstacleHeight.grid(row=rosRows+8, column = first_col)

	et_obstacleRadius = Entry(root, textvariable=obstacle_radius_input, width=22)
	et_obstacleRadius.grid(row=rosRows+8, column = second_col)
	
	# starts the gui
	mainloop()
	
if __name__ == "__main__":
	main()
