#!/usr/bin/env python

import rospy, mavros
import math
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from custom_msgs.msg import StateMachine

takeoff_state = None
flight_state = None
current_state = None
current_pos = None
destination = None


def callback(data):
	
	global takeoff_state, flight_state
	takeoff_state = data.takeoff
	flight_state = data.flight
	# print(str(preflight_state))

def getCurrentState(data):

	global current_state
	current_state = data

def getCurrentPosition(data):

	global current_pos
	current_pos = data
	
def getDestination(data):
	global destination
	destination = data

if __name__=='__main__':

	rospy.init_node('takeoff_node', anonymous=True)

	rospy.Subscriber("/state_machine/state", StateMachine, callback)
	rospy.Subscriber("/mavros/state", State, getCurrentState)
	rospy.Subscriber('/mavros/local_position/odom', Odometry, getCurrentPosition)
	
	rospy.Subscriber("/lakitu/flight_target", PoseStamped, getDestination)
	
	state_pub = rospy.Publisher('/state_machine/state', StateMachine, queue_size=100)
	local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=100, latch=True)
	
	setModeSrv = rospy.ServiceProxy("/mavros/set_mode", SetMode) #http://wiki.ros.org/mavros/CustomModes
	
	rate = rospy.Rate(60)
	
	# select desired initial height (needs to be a float)
	height = float(2)

	#StateMachine msg that will switch Lakitu to 'hover' state
	state = StateMachine()
	state.preflight = False
	state.takeoff = False
	state.flight = True
	state.hover = False
	state.land = False
	state.emergency = False
	state.manual = False
	
	while current_pos == None and not rospy.is_shutdown():
		continue
		
	start_pos = current_pos
	
	# create pose destination based on desired height
	pose = PoseStamped()
	pose.pose.position.x = start_pos.pose.pose.position.x
	pose.pose.position.y = start_pos.pose.pose.position.y
	pose.pose.position.z = height


	while not rospy.is_shutdown():
		
		#takeoff node does nothing if not in takeoff state
		if(takeoff_state == None):
			continue
		#shouldn't do anything if current_state isn't reading	
		if(current_state == None):
			continue
		if(current_pos == None):
			continue	
		if(flight_state):
			continue	
		if(destination == None):
			continue
		
				

		if(takeoff_state):

			#sets FCU mode to offboard, should also takeoff to z=2
			if(current_state.mode != "OFFBOARD"):
				setModeSrv(0, 'OFFBOARD')
			
			# once we have reached the desired height, switch to desired state
			if(current_pos.pose.pose.position.z >= start_pos.pose.pose.position.z + height - float(0.1))\
			 and (current_pos.pose.pose.position.z >= start_pos.pose.pose.position.z + height + float(0.1)):
			 	state_pub.publish(state)
			 
			local_pos_pub.publish(pose)
			
		rate.sleep()		
		
		
		
		
