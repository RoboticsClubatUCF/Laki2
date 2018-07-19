#!/usr/bin/env python

import rospy, mavros
import math
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, TwistStamped
from custom_msgs.msg import StateMachine

preflight_state = None
current_state = None

def callback(data):
	
	global preflight_state
	preflight_state = data.preflight
	# print(str(preflight_state))

def getCurrentState(data):

	global current_state
	current_state = data

if __name__=='__main__':

	#initialize node, subscribe to state
	rospy.init_node('preflight_node', anonymous=True)	
	
	rospy.Subscriber("/state_machine/state", StateMachine, callback)
	rospy.Subscriber("/mavros/state", State, getCurrentState)
	
	local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=100)
	state_pub = rospy.Publisher('/state_machine/state', StateMachine, queue_size=100)
	
	#service proxies for arming and setting mode
	armCommandSrv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
	setModeSrv = rospy.ServiceProxy("/mavros/set_mode", SetMode) #http://wiki.ros.org/mavros/CustomModes

	#msgs sent at 60hz
	rate = rospy.Rate(60)

	#creating pose msg
	pose = PoseStamped()
	pose.pose.position.x = 0
	pose.pose.position.y = 0
	pose.pose.position.z = 0

	#StateMachine msg that will switch Lakitu to 'takeoff' state
	state = StateMachine()
	state.preflight = False
	state.takeoff = True
	state.flight = False
	state.hover = False
	state.land = False
	state.emergency = False
	state.manual = False

	last_request = rospy.Time.now()

	#main loop of program
	while not rospy.is_shutdown():

		if(preflight_state is None):
			continue
		
		if(current_state is None):
			continue

		if(preflight_state):

			# local_pos_pub.publish(pose)
			if(not current_state.armed): 
				armCommandSrv(True)

		if(current_state.armed and preflight_state):
			state_pub.publish(state)
		
		rate.sleep()