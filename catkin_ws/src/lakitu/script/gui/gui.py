from tkinter import *
import cv2, numpy as np, mavros, rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
# for getGpsImage:
import Image, urllib, StringIO
from math import log, exp, tan, atan, pi, ceil

from Server import Server # for interfacing with Server

Server = Server()

# variables updated by ROS (initialized here to ensure global access)
rosGpsData = None
latitude = None
longitude = None
current_pos = None
root = None

# global variables needed for gui objects
label_gpsData = None
label_imuData = None

# relates to the image the gui displays of the GPS area
pixel_height = None
pixel_width = None

canvas = None

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
	#extract the information from ROS that we need (latitude and longitude)
	global latitude
	global longitude
	
	if latitude is not None:
		gpsString = str(latitude) + ", " + str(longitude)
	
		label_gpsData.config(text=gpsString)
		#label_imuData.config(text=current_pos)
	
	
	root.after(100, update)

# THIS METHOD IS A TEST OF UPDATING THE GUI, USE THIS TEMPLATE AS NEEDED
# updates the label after a certain amount of time to counter 
counter = 0
def my_after(): 
	global counter
	counter = counter + 1	
	print counter
	new_text = counter 
	
	#change text of label
	label_gpsData.config(text=new_text)
	
	# call again after 100 ms
	root.after(100, my_after)
# you must call the method at least once to get it started, it will continue on its own afterwards
#my_after()


# converts gps coordinates to pixel coordinates
#def GpsToPixel(gps):
	
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


	print "\nmaking line from (gps) = " + str(gps1) + " to " + str(gps2) + ")"
	print "making line from (pixel) = (" + str(x1) + ", " + str(y1) + ") to (" + str(x2) + ", " + str(y2) + ")"

	line = canvas.create_line(x1,y2,x2,y1, fill=color)

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
	#print str((gps)) + " " + str((imageSize)) + " " + str((A)) + " " +  str((B))
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

def clickCallback(event):
	pix = (event.y, event.x)
	print "clicked at", event.x, event.y
	print "clicked gps = ", pixelToGps((event.x, event.y), "x"), pixelToGps((event.x,event.y), "y")
	lon = pixelToGps(pix, "x")
	lat = pixelToGps(pix, "y")

	makeCircle((lat, lon), 15, "orange")

	print "waypoints1 = " + str(Server.mission_waypoints)
	Server.add_mission_waypoint((lat, lon), 0)
	print "waypoints2 = " + str(Server.mission_waypoints)
	

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

# gets the gps image from google with specified latitude and longitude coords
def getGpsImage(upperLeftCoords, lowerRightCoords):
	print("getting GPS image for the following coords:")
	print upperLeftCoords
	print lowerRightCoords
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
	
	#upperleft =  '28.587011, -81.200438'  
	#lowerright = '28.582682, -81.195694'

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
		    print x, y, position
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
	lat = gpsToPixel(centroid[0], "lat")
	lon = gpsToPixel(centroid[1], "lon")

	TL = [lon-radius, lat+radius] # top left coordinates
	BR = [lon+radius, lat-radius] # top right coordinates

	#TL = [centroid[1]-radius, centroid[0]+radius] # top left coordinates
	#BR = [centroid[1]+radius, centroid[0]-radius] # top right coordinates

	#id = C.create_oval(x0, y0, x1, y1, option, ...)
	global canvas


	canvas.create_oval(TL[0], TL[1], BR[0], BR[1], fill=color)

def main():
	global root
	global label_gpsData
	global label_imuData
	global latitude
	global longitude
	global pixel_height
	global pixel_width
	global canvas

	# initialize ros nodes
	# init_ros() 

	#initialize a Tkinter object
	root = Tk()
	
	# create a frame and tell it which Tkinter object we are using
	frame = Frame(root)
	
	
	# default gps for simulator:
	# latitude: 47.3977417
	# longitude: 8.5455941

	'''
	while(latitude == None):
		continue
	imageLat1 = latitude - 0.0025
	imageLat2 = latitude + 0.0025
	imageLong1 = longitude - 0.01
	imageLong2 = longitude + 0.01
	
	imgLowerRight = str(imageLat1) + ", " + str(imageLong1)
	imgUpperLeft  = str(imageLat2) + ", " + str(imageLong2)
	
	print(imgUpperLeft)
	print(imgLowerRight)

	'''

	#getGpsImage('28.587011, -81.200438', '28.582682, -81.195694')
	#getGpsImage('47.3977413, -8.545594', '47.3927413, -8.535594')
	
	str_corners = getCorners(1)
	
	#getGpsImage(str_corners[0], str_corners[3])

	# get image data (gps satellite data)
	path = "gps.png"
	img = PhotoImage(file=path) # used for Tkinter canvas object
	cv_image = cv2.imread(path) # used to get pixel measurements
	pixel_height = np.size(cv_image, 0) # height of image in pixels
	pixel_width = np.size(cv_image, 1)  # width of image in pixels

	print "height = " + str(pixel_height)
	print "width = " + str(pixel_width)

	#display an image from file path
	canvas = Canvas(root, width = pixel_width, height = pixel_height)
	canvas.grid(row=0, column=0, sticky=W,rowspan=10, columnspan=10)
	canvas.create_image(0,0, anchor = NW, image = img)
	canvas.bind("<Button-1>", clickCallback)

	#Label experimenting
	#label that provides info text
	label_gpsName = Label(root, text="GPS coordinates = ")
	label_gpsName.grid(row=0, column=11, sticky = N)

	#label that updates based on given gps data
	label_gpsData = Label(root, text="NO GPS DATA")
	label_gpsData.grid(row=0, column=12, sticky = N)

	''' label for imu data (temporarily removed until we know what data is useful'''
	#label_imuName = Label(root, text="orientation = ")
	#label_imuName.grid(row=1,column=0, sticky = W)

	#label_imuData = Label(root, text="NO IMU DATA")
	#label_imuData.grid(row=1, column=1)
	
	corners = getCorners(0)

	Bx = corners[1][1]
	By = corners[1][0]
	Ax = corners[0][1]
	Ay = corners[2][0]

	missions = Server.get_missions()

	# list of waypoints outlc
	flyzone_waypoints = missions[0].fly_zones[0].boundary_pts
	
	print corners
	lat = gpsToPixel(corners[3][0], "lat")
	lon = gpsToPixel(corners[3][1], "lon")

	print "lat = " + str(lat)
	print "lon = " + str(lon)

	#makeCircle((lat, lon), 50)
	
	for i in range(0, len(flyzone_waypoints)):
		lat = flyzone_waypoints[i].latitude
		lon = flyzone_waypoints[i].longitude

		pix_lat = gpsToPixel(lat, "lat")
		pix_lon = gpsToPixel(lon, "lon")
		#makeCircle((pix_lat, pix_lon), 20)
		makeCircle((lat, lon), 20, 'red')
		if(i != len(flyzone_waypoints)-1):
			createLine((lat, lon), (flyzone_waypoints[i+1].latitude, flyzone_waypoints[i+1].longitude), "red")
		else:
			createLine((lat, lon), (flyzone_waypoints[0].latitude, flyzone_waypoints[0].longitude), "red")


	search_grid_waypoints = missions[0].search_grid_points
	
	for i in range(0, len(search_grid_waypoints)):
		makeCircle((search_grid_waypoints[i].latitude, search_grid_waypoints[i].longitude),20, "green")

	mission_waypoints = missions[0].mission_waypoints
	for i in range(0, len(mission_waypoints)):
		makeCircle((mission_waypoints[i].latitude, mission_waypoints[i].longitude),20, "blue")
		#createLine((mission_waypoints[i].latitude, mission_waypoints[i].longitude), "blue")

	update() # update the gui with new info	
	
	
	# starts the gui
	mainloop()
	
if __name__ == "__main__":
	main()
