#!/usr/bin/env python

import cmd, sys, threading
import rospy
from mavros_msgs.srv import ParamSet, ParamGet, SetMode, CommandBool
from mavros_msgs.msg import RCIn

from geometry_msgs.msg import Point
from laki2_msg.msg import MissionPath

from laki2_common import TextColors

rc_msg_type = None
rc_kill = False
  
class Value():

	def __init__(self, integer, real):
		self.integer = integer
		self.real = real

# simple Waypoint (WP) class to hold (x,y,z)s 
class WP:

	def __init__(self, x, y, z):
		self.x = x
		self.y = y
		self.z = z

	def toPoint(self):
	
		point = Point()
		point.x = self.x
		point.y = self.y
		point.z = self.z	

		return point

	def isEqual(self, wp): #is this waypoint (self) equal to another waypoint (wp)
		
		return (self.x == wp.x and self.y == wp.y and self.z == wp.z)

# simple GPS coordinate class
# coords are used as mission points in MISSION state
class Coord:

	def __init__(self, latitude, longitude, altitude=0):
		self.latitude = latitude
		self.longitude = longitude
		self.altitude = altitude	

# a thread that publishes RC Controller messages to override the default mavros ones
class RC_Controller():

	def __init__(self):

		self.rc_pub = rospy.Publisher('/mavros/rc/in', RCIn, queue_size = 100)

	def _publish_RC(self):

		global rc_kill
		global rc_msg_type

		rate = rospy.Rate(30)

		rc_msg = RCIn()

		while not rospy.is_shutdown():

			rate.sleep()

			if rc_kill:
				break 

			# print '...'
				
			if rc_msg_type is None:	
				rc_msg.channels = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
			elif rc_msg_type == 'takeoff':
				rc_msg.channels = [0,0,0,1500,2067,0,2113,0,0,0,0,0,0,0,0,0,0,0]
			elif rc_msg_type == 'standby':
				rc_msg.channels = [0,0,0,1500,961,0,2113,0,0,0,0,0,0,0,0,0,0,0]		

			self.rc_pub.publish(rc_msg)

		return 

	def publish_RC(self):
		threading.Thread(target=self._publish_RC).start()			

class SimShell(cmd.Cmd):

	intro = 'Testing shell for Laki2_sim'
	prompt = '(Laki2)'
	file = None

	# uses ROS service to set ARDUPILOT mode
	# takes string with mode name, ALL CAPS
	# made because this method is used multiple times throughout
	def setMode(self, mode):

		try:	#service call to set mode to auto
			setModeSrv = rospy.ServiceProxy("/mavros/set_mode", SetMode) #http://wiki.ros.org/mavros/CustomModes
			setModeResponse = setModeSrv(0, mode)
			rospy.loginfo(TextColors.OKGREEN + str(setModeResponse) + TextColors.ENDC)
		except rospy.ServiceException, e:
			rospy.loginfo(TextColors.FAIL + 'Service call failed: %s' %e + TextColors.ENDC)

		return setModeResponse	

	def do_arm(self, arg):
		'Attempts to arm the copter'

		try:
			armCommandSrv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)		
			armResponse = armCommandSrv(True)
			print armResponse
		except rospy.ServiceException, e:
			rospy.loginfo(TextColors.FAIL + 'Service call failed: %s' %e + TextColors.ENDC)	
				

	def do_kill(self, arg):
		'Kills sensor named by arg'
		
		if arg == 'rc':
			param = 'SIM_RC_FAIL'
			value = Value(1, 0)
		elif arg == 'gps':
			param = 'SIM_GPS_DISABLE'
			value = Value(1,0)	
		set_param(param, value)

	def do_revive(self, arg):	
		'Un-kills sensors named by arg'

		if arg == 'rc':
			param = 'SIM_RC_FAIL'
			value = Value(0, 0)
		elif arg == 'gps': #gps status on/off 0/-1
			param = 'SIM_GPS_DISABLE'
			value = Value(0,0)	
		set_param(param, value)	

	def do_rc(self, arg):
		'Takeoff to a constant height'

		global rc_msg_type
		rc_msg_type = str(arg)

	def do_mission(self, arg):
		'Push a test mission to fly'

		mission_path = MissionPath()
		wp1 = WP(10,10,10).toPoint()
		wp2 = WP(20,20,10).toPoint()
		wp3 = WP(30,30,10).toPoint()
		wp4 = WP(40,40,10).toPoint()
		wp5 = WP(50,50,10).toPoint()
		mission_path.points = [wp1,wp2,wp3,wp4,wp5]

		mission_pub = rospy.Publisher('/laki2/mission/waypoints', MissionPath, queue_size = 100)

		mission_pub.publish(mission_path)

	def do_mode(self, arg):
		'Set flight mode'

		if arg == 'auto':
			self.setMode('AUTO')
		elif arg == 'land':
			self.setMode('LAND')	
		elif arg == 'brake':
			self.setMode('BRAKE')	
		elif arg == 'stabilize':
			self.setMode('STABILIZE')	

	def do_end(self, arg):
		'End the sim-shell'

		global rc_kill

		input_rtl = raw_input('do you want to RTL? (y/n) : ')
		if(input_rtl == 'y'):
			self.setMode('RTL')

		rc_kill = True
		return True	

def set_param(param, value):
	
	setParamSrv = rospy.ServiceProxy('/mavros/param/set', ParamSet)

	try:
		response = setParamSrv(param, value)
		print(response)	
	except rospy.ServiceException, e:
		rospy.loginfo('Service call failed: %s' %e)		

if __name__=='__main__':

	rospy.init_node('sim_shell', anonymous=True)
	rc_cont = RC_Controller()
	rc_cont.publish_RC()
	SimShell().cmdloop('Interactive testing suite for Laki2_sim')
	print 'all done'
