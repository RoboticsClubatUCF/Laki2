#!/usr/bin/env python

import rospy, mavros
import math, smach, smach_ros
from mavros_msgs.srv import CommandBool, SetMode, ParamGet, CommandTOL
from mavros_msgs.msg import State, RCIn
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from laki2_common import TextColors

# uses ROS service to set ARDUPILOT mode
# takes string with mode name, ALL CAPS e.g. 'BRAKE' or 'AUTO'
# TESTED WORKING: AUTO, LAND, BRAKE
def setMode(mode):

	try:	#service call to set mode to auto
		setModeSrv = rospy.ServiceProxy("/mavros/set_mode", SetMode) #http://wiki.ros.org/mavros/CustomModes
		setModeResponse = setModeSrv(0, mode)
		rospy.loginfo(TextColors.OKGREEN + str(setModeResponse) + TextColors.ENDC)
	except rospy.ServiceException, e:
		rospy.loginfo(TextColors.FAIL + 'Service call failed: %s' %e + TextColors.ENDC)

	return setModeResponse	    

class MonitorGPS(smach.State):

	def gps_status_cb(self, data):
		self.gps_status = data.status

	def __init__(self):
		smach.State.__init__(self,outcomes=['done', 'reset'])
		rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_status_cb) 
		self.gps_status = None
		pub = rospy.Publisher('/laki2/status/gps', String)

	def execute(self, userdata):

		rate = rospy.Rate(30)

		while not rospy.is_shutdown():

			rate.sleep()

			if self.gps_status.status == -1:
				pub.Publish('fuck')

		return 'done'	
