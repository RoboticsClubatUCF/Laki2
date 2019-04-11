#!/usr/bin/env python

import rospy, mavros
import math, smach, smach_ros
from mavros_msgs.srv import CommandBool, SetMode, ParamGet, CommandTOL
from mavros_msgs.msg import State, RCIn
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped

from laki2_common import TextColors

import flight, monitor

''' the main states for Laki2's state machine (SMACH)'''

# initializing global variables to NONE
rcNum = None
current_state = None
current_pos = None
# target = None
# gotTarget = False

# callback that reads the /mavros/state topic
def getMavrosState(data):

	global current_state
	current_state = data	

# callback that reads the /mavros/rc/in topic
# /mavros/rc/in is an RCIn topic, which provides an array of RC channels
# provided array can be used to simulate a real-world rc unit
def getRCChannels(data):

	global rcNum
	rcNum = data.channels[4] #channels[4] == 'g' switch on RC controller (2067|961 -> ON|OFF)

# callback that reads the /laki2/flight_target/local topic
# flight_target/local is ideally published by the path-planning system to guide Laki2 
# def getLocalTarget(data):

# 	global target
# 	global gotTarget
# 	gotTarget = True
# 	target = data

# reads current position from /mavros/local_position/odom
def getCurrentPosition(data):

	global current_pos
	current_pos = data	


# uses ROS service to set ARDUPILOT mode
# takes string with mode name, ALL CAPS
# made because this method is used multiple times throughout
def setMode(mode):

	try:	#service call to set mode to auto
		setModeSrv = rospy.ServiceProxy("/mavros/set_mode", SetMode) #http://wiki.ros.org/mavros/CustomModes
		setModeResponse = setModeSrv(0, mode)
		rospy.loginfo(TextColors.OKGREEN + str(setModeResponse) + TextColors.ENDC)
	except rospy.ServiceException, e:
		rospy.loginfo(TextColors.FAIL + 'Service call failed: %s' %e + TextColors.ENDC)

	return setModeResponse	

# SM State: Land
# From:		Mission, Hover(NYI)
# Purpose:	controlled landing of the quad wherever it is
#			NO RTL 
class Land(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['returnToPREFLIGHT','exit_flight'])

	def execute(self,userdata):

		setMode('LAND')

		while not rospy.is_shutdown():

			if(current_pos.pose.pose.position.z == 0):
				return 'returnToPREFLIGHT'		

# SM State: PREFLIGHT
# From: 	NONE
# Purpose: 	basic preflight checks (working odometry, connection to FCU)
#			listens for start switch to begin flight			
class Preflight(smach.State):

	'''PREFLIGHT state houses all 'preflight checks' and sets up for following states, always comes first'''

	def __init__(self):
		# smach.State.__init__(self, outcomes=['toTAKEOFF','toFLIGHT'])
		smach.State.__init__(self, outcomes=['exit','toTAKEOFF'])
		
	def execute(self, userdata):

		rate = rospy.Rate(30) #runs following loop at 30hz

		#flags to stop constant output of warning messages
		targetFlag = True
		readyFlag = True
		noOdomFlag = True
		testFlag = False

		while not rospy.is_shutdown():

			rate.sleep()
			
			# does nothing if mavros is not up and running yet (/mavros/state)
			if (current_state is None):
				rospy.loginfo(TextColors.WARNING + 'MAVROS NOT RUNNING' + TextColors.ENDC)
				continue

			# /mavros/local_position/odom	
			if (current_pos is None):
				rospy.loginfo(TextColors.WARNING + 'NO ODOMETRY DATA' + TextColors.ENDC)
				continue	
				
			# if (current_pos is None and noOdomFlag):
			# 	rospy.loginfo(TextColors.WARNING + 'NO ODOMETRY DATA' + TextColors.ENDC)
			# 	noOdomFlag = False
			# 	continue	

			if (readyFlag):	
				rospy.loginfo(TextColors.OKBLUE + 'READY WHEN YOU ARE' + TextColors.ENDC)	
				readyFlag = False

			# #does nothing if no target is published	(/laki2/flight_target)
			# if (not gotTarget):
			# 	continue	

			# listens for RC switch, if ON (2067) switches to TAKEOFF state
			# rcNum is from function getRCChannels()
			if (rcNum == 2067): 
				rospy.loginfo(TextColors.OKGREEN + "RC SWITCH: ON" + TextColors.ENDC)
				return 'toTAKEOFF'

			# if testFlag:	
				


# SM State: TAKEOFF
# From: 	PREFLIGHT
# Purpose:	to takeoff from the ground to a hardcoded height
class Takeoff(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['exit','toPREFLIGHT','toFLIGHT_SM', 'reset'])

	def execute(self, userdata):

		rate = rospy.Rate(30)

		if(current_state.mode == "GUIDED"):
			guided = True
		else:
			guided = False	

		takeoffFlag = True	

		# while not rospy.is_shutdown():

		# 961 is up on switch 'g'
		# if (rcNum == 961):

		# 	if(current_state.mode != 'STABILIZE'):
		# 		setMode("STABILIZE")

			# continue

		if (current_state.mode != 'GUIDED'):	
			try:	#service call to set mode to guided (necessary for takeoff)
				setModeSrv = rospy.ServiceProxy("/mavros/set_mode", SetMode) #http://wiki.ros.org/mavros/CustomModes
				setModeResponse = setModeSrv(0, 'GUIDED')
				rospy.loginfo(TextColors.OKGREEN + str(setModeResponse) + TextColors.ENDC)
				guided = True

			except rospy.ServiceException, e:
				rospy.loginfo(TextColors.FAIL + 'Service call failed: %s' %e + TextColors.ENDC)

		# arming Laki2, must be armed before takeoff (PX4 and ARDUPILOT)
		if ((not current_state.armed) and guided): 
			armCommandSrv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)		
			armResponse = armCommandSrv(True)
			# rospy.loginfo(armResponse)		

		if (current_state.armed and guided and takeoffFlag):
			takeoffCommandSrv = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
			takeoffResponse = takeoffCommandSrv(0.0,0.0,0,0,1.0)

			if takeoffResponse.success == 'True':
				takeoffFlag = False
				return 'toFLIGHT_SM'

		
		return 'reset'	
			# rospy.loginfo(takeoffResponse)

		# # this hard-coded altitude needs to die, eventually	
		# if (current_pos.pose.pose.position.z <= 1.1 and current_pos.pose.pose.position.z >= 0.9):	
		# 	# setMode('STABILIZE')
		# 	rospy.loginfo('got there, bitch')
		

		rate.sleep()	

def main():
	
	rospy.init_node('laki2_sm', anonymous=True)

	rospy.Subscriber("/mavros/state", State, getMavrosState) #provides mavros state (connected, flight mode, etc.)
	rospy.Subscriber("/mavros/rc/in", RCIn, getRCChannels) #subscribe to mavros topic for RC Data
	rospy.Subscriber('/mavros/local_position/odom', Odometry, getCurrentPosition) #gives current, continuous position (odom frame)

	sm = smach.StateMachine(outcomes=['exit_sm'])

    # Open the container, declare the SMACH states
	with sm:

		smach.StateMachine.add('PREFLIGHT', Preflight(), transitions={'toTAKEOFF':'TAKEOFF','exit':'exit_sm'})
		smach.StateMachine.add('TAKEOFF', Takeoff(), transitions={'toPREFLIGHT': 'PREFLIGHT','toFLIGHT_SM':'FLIGHT_SM','exit':'exit_sm', 'reset':'TAKEOFF'})

		flight_sm = smach.StateMachine(outcomes=['exit_to_preflight','exit_flight_sm'])
		# monitor_sm = smach.Concurrence(outcomes={'done', 'reset'},default_outcome='done', child_termination_cb=child_term_cb, outcome_cb=out_cb)

		# with monitor_sm:
		# 	smach.Concurrence.add('MONITOR_GPS', monitor.MonitorGPS())
			#space here for more sensor monitors

		with flight_sm:
			
			smach.StateMachine.add('MISSION', flight.Mission(), transitions={'returnToPREFLIGHT':'exit_to_preflight','toLAND':'LAND','exit_flight':'exit_flight_sm'})
			smach.StateMachine.add('LAND', Land(), transitions={'returnToPREFLIGHT':'exit_to_preflight','exit_flight':'exit_flight_sm'})
			# smach.StateMachine.add('MONITOR', monitor_sm, transitions={'done':'exit_to_preflight', 'reset':'MONITOR'})

		smach.StateMachine.add('FLIGHT_SM', flight_sm, transitions={'exit_to_preflight':'PREFLIGHT','exit_flight_sm':'exit_sm'})
		# smach.StateMachine.add('MONITOR_SM', monitor_sm, transitions={'done':'exit_sm', 'reset':'MONITOR_SM'})	

	introspect = smach_ros.IntrospectionServer('server', sm, '/SM')
	introspect.start()

	#to run SMACH Viewer: rosrun smach_viewer smach_viewer.py

	outcome = sm.execute()	
	
if __name__ == '__main__': 
    main() 