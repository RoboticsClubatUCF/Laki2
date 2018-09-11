#!/usr/bin/env python

import rospy, mavros
import math
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from custom_msgs.msg import StateMachine

flight_state = None
current_state = None
current_pos = None

def callback(data):
	
	global flight_state
	flight_state = data.flight
	print(str(flight_state))

def getCurrentState(data):

	global current_state
	current_state = data

def getCurrentPosition(data):

	global current_pos
	current_pos = data


if __name__=='__main__':

	rospy.init_node('flight_node', anonymous=True)

	rospy.Subscriber("/state_machine/state", StateMachine, callback)
	rospy.Subscriber("/mavros/state", State, getCurrentState)
	rospy.Subscriber('/mavros/local_position/odom', Odometry, getCurrentPosition)
	local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=100)
	state_pub = rospy.Publisher('/state_machine/state', StateMachine, queue_size=100)
	#service proxies for arming and setting mode
	armCommandSrv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
	setModeSrv = rospy.ServiceProxy("/mavros/set_mode", SetMode) #http://wiki.ros.org/mavros/CustomModes

	#msgs sent at 60hz
	rate = rospy.Rate(10)

	#creating pose msg
	pose = PoseStamped()
	pose.pose.position.x = 0
	pose.pose.position.y = 0
	pose.pose.position.z = .5

	#StateMachine msg that will switch Lakitu to 'hover' state
	state = StateMachine()
	state.preflight = False
	state.takeoff = False
	state.flight = False
	state.hover = True
	state.land = False
	state.emergency = False

	# last_request = rospy.Time.now()

	#main loop of program
	while not rospy.is_shutdown():

		if(flight_state is None):
			# print('what the fuck')
			continue
		if(current_state is None):
			continue
		if(current_pos is None):
			continue	

		if(flight_state):
			local_pos_pub.publish(pose)	

		#if(current_pos.pose.pose.position.x >= 29.9 ) and (current_pos.pose.pose.position.x <= 30.1):
			#state_pub.publish(state)
	
		rate.sleep()