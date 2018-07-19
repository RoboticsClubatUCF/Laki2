#!/usr/bin/env python

import rospy, mavros
import math
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, TwistStamped

current_state = None

def getCurrentState(data):
	global current_state
	current_state = data
	

if __name__=='__main__':

	rospy.init_node("state_machine", anonymous=True)
	rospy.Subscriber("/mavros/state", State, getCurrentState)
	local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=100)
	cmdVelPub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=100)
	# cmdVelListener = rospy.subscriber("/mavros/setpoint_velocity/cmd_vel", TwistStamped, )
	armCommandSrv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
	setModeSrv = rospy.ServiceProxy("/mavros/set_mode", SetMode) #http://wiki.ros.org/mavros/CustomModes

	rate = rospy.Rate(60)

	pose = PoseStamped()
	pose.pose.position.x = 0
	pose.pose.position.y = 0
	pose.pose.position.z = 2

	twist = TwistStamped()
	twist.twist.linear.x = 0
	twist.twist.linear.y = 0
	twist.twist.linear.z = 2

	twist.twist.angular.x = 0
	twist.twist.angular.y = 0
	twist.twist.angular.z = 0

	last_request = rospy.Time.now()

	while not rospy.is_shutdown():
		if(current_state is None ):
			continue
		if(not current_state.armed and (rospy.Time.now() -last_request > rospy.Duration(5.0))):
			armCommandSrv(True)			
		if(current_state.mode is not "OFFBOARD" and (current_state.armed is True)):
			setModeSrv(0, 'OFFBOARD')
		# armCommandSrv(True)	
		local_pos_pub.publish(pose)
		rate.sleep()