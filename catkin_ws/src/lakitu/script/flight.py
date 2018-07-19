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
destination_pos = None

def callback(data):
	
	global flight_state
	flight_state = data.flight
	# print(str(flight_state))

def getCurrentState(data):

	global current_state
	current_state = data

def getCurrentPosition(data):

	global current_pos
	current_pos = data
	
def getDestination(data):
	global destination
	destination = data

def flightCallback(data):
	global destination
	destination = data
	local_pos_pub.publish(data)

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
	rate = rospy.Rate(60)


	#StateMachine msg that will switch Lakitu to 'hover' state
	state = StateMachine()
	state.preflight = False
	state.takeoff = False
	state.flight = False
	state.hover = True
	state.land = False
	state.emergency = False
	state.manual = False

	destination = PoseStamped()
	destination.pose.position.x = 0
	destination.pose.position.y = 0
	destination.pose.position.z = 0

	# last_request = rospy.Time.now()

	# flag = True
	#main loop of program
	while not rospy.is_shutdown():

		if(flight_state is None):
			continue
		if(current_state is None):
			continue
		if(current_pos is None):
			continue	

		if(flight_state):

			rospy.Subscriber("/lakitu/flight_target", PoseStamped, flightCallback)

			if(current_state.mode != "OFFBOARD"):
				setModeSrv(0, 'OFFBOARD')
			# local_pos_pub.publish(destination)	


			#this block of code causes strange behavior	
			if(current_pos.pose.pose.position.x >= destination.pose.position.x - float(0.1) )\
			  and (current_pos.pose.pose.position.x <= destination.pose.position.x + float(0.1))\
			  and (current_pos.pose.pose.position.y >= destination.pose.position.y - float(0.1))\
			  and (current_pos.pose.pose.position.y <= destination.pose.position.y + float(0.1))\
			  and (current_pos.pose.pose.position.z >= destination.pose.position.z - float(0.1))\
			  and (current_pos.pose.pose.position.z <= destination.pose.position.z + float(0.1)):
			 	state_pub.publish(state)
	
		rate.sleep()
