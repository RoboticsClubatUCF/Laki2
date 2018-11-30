#!/usr/bin/env python

import rospy, mavros
import math, smach, smach_ros
from mavros_msgs.srv import CommandBool, SetMode, ParamGet
from mavros_msgs.msg import State, RCIn
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped

# an attempt at a hierarchical state machine FLIGHT within main sm
# abject failure


class Flight(smach.State):


	'''A flight mode that allows for changing of flight targets midair'''

	def __init__(self):
		smach.State.__init__(self, outcomes=[])
		self.target_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=100)
		# self.target_pub = None
		
	def setpoint_flight(self,target):
			
		self.target_pub.publish(target)

	def execute(self, userdata):
		
		rospy.loginfo("FLIGHT REACHED")

		rate = rospy.Rate(30)

		rospy.Subscriber("/lakitu/flight_target/local", PoseStamped, getLocalTarget) #tells where we're going

		while not rospy.is_shutdown():

			if ((current_state.mode != 'OFFBOARD') and (gotTarget) and current_state.armed):	
				try:	#service call to set mode to OFFBOARD
					setModeSrv = rospy.ServiceProxy("/mavros/set_mode", SetMode) #http://wiki.ros.org/mavros/CustomModes
					setModeResponse = setModeSrv(0, 'OFFBOARD')
					# rospy.loginfo(str(setModeResponse) + '\nMODE: %s' % current_state.mode)

				except rospy.ServiceException, e:
					rospy.loginfo('Service call failed: %s' %e)

			rate.sleep()
