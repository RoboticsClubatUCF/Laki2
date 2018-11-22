#!/usr/bin/env python

import rospy, mavros
import math, smach, smach_ros
from mavros_msgs.srv import CommandBool, SetMode, ParamGet, CommandTOL
from mavros_msgs.msg import State, RCIn
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped

from laki2_common import TextColors

import flight

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
# provided array can be used to simulate a real-world rc controller
def getRCChannels(data):

	global rcNum
	rcNum = data.channels[6] #channels[6] is the ON/OFFswitch (2113 is ON/ 1000 is OFF)	

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

		while not rospy.is_shutdown():

			rate.sleep()
			
			# does nothing if mavros is not up and running yet (/mavros/state)
			if (current_state is None):
				rospy.loginfo(TextColors.WARNING + 'MAVROS NOT RUNNING' + TextColors.ENDC)
				continue

			if (current_pos is None):
				continue	
				
			if (current_pos is None and noOdomFlag):
				rospy.loginfo(TextColors.WARNING + 'NO ODOMETRY DATA' + TextColors.ENDC)
				noOdomFlag = False
				continue	

			if (readyFlag):	
				rospy.loginfo(TextColors.OKBLUE+ 'READY WHEN YOU ARE' + TextColors.ENDC)	
				readyFlag = False

			# #does nothing if no target is published	(/laki2/flight_target)
			# if (not gotTarget):
			# 	continue	

			# if (gotTarget and targetFlag):
			# 	rospy.loginfo(TextColors.OKGREEN + 'TARGET RECEIVED' + TextColors.ENDC)	
			# 	targetFlag = False	

			# listens for RC switch, if ON (2113) switches to TAKEOFF state
			# rcNum is from function getRCChannels()
			if rcNum == 2113: 
				rospy.loginfo(TextColors.OKGREEN + "RC SWITCH: ON" + TextColors.ENDC)
				return 'toTAKEOFF'


# SM State: TAKEOFF
# From: 	PREFLIGHT
# Purpose:	to takeoff from the ground to a hardcoded height of ten meters
class Takeoff(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['exit','toPREFLIGHT','toFLIGHT_SM'])


	#NOT WORKING FOR ARDUPILOT	
	def getTakeoffAlt(self):

		'''reading the PX4 param 'MIS_TAKEOFF_ALT'
		useful method to read any PX4 param in script
		https://docs.px4.io/en/advanced_config/parameter_reference.html'''

		try:
			takeoffAltSrv = rospy.ServiceProxy('/mavros/param/get', ParamGet)	
			takeoffResponse = takeoffAltSrv('MIS_TAKEOFF_ALT')
			self.targetAlt = takeoffResponse.value.real

		except rospy.ServiceException, e:
			rospy.loginfo('Service call failed: %s' %e)	

	def execute(self, userdata):

		rate = rospy.Rate(30)

		if(current_state.mode == "GUIDED"):
			guided = True
		else:
			guided = False	

		takeoffFlag = True	

		# startAlt = current_pos.pose.pose.position.z	

		while not rospy.is_shutdown():

			# rospy.loginfo(TextColors.FAIL + current_state.mode)

			if (current_state.mode != 'GUIDED'):	
				try:	#service call to set mode to takeoff
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
				takeoffResponse = takeoffCommandSrv(0.0,0.0,0,0,10.0)

				if takeoffResponse.success == 'True':
					takeoffFlag = False
				
				# rospy.loginfo(takeoffResponse)

			# this hard-coded altitude needs to die, eventually	
			if (current_pos.pose.pose.position.z <= 10.1 and current_pos.pose.pose.position.z >= 9.9):	
				return 'toFLIGHT_SM'

			rate.sleep()	


def main():
	
	rospy.init_node('laki2_sm', anonymous=True)

	rospy.Subscriber("/mavros/state", State, getMavrosState) #provides mavros state (connected, flight mode, etc.)
	rospy.Subscriber("/mavros/rc/in", RCIn, getRCChannels) #subscribe to mavros topic for RC Data
	# rospy.Subscriber("/laki2/flight_target/local", PoseStamped, getLocalTarget) #tells where we're going
	rospy.Subscriber('/mavros/local_position/odom', Odometry, getCurrentPosition) #gives current, continuous position

	sm = smach.StateMachine(outcomes=['exit_sm'])

    # Open the container, declare the SMACH states
	with sm:

		smach.StateMachine.add('PREFLIGHT', Preflight(), transitions={'toTAKEOFF':'TAKEOFF','exit':'exit_sm'})
		smach.StateMachine.add('TAKEOFF', Takeoff(), transitions={'toPREFLIGHT': 'PREFLIGHT','toFLIGHT_SM':'FLIGHT_SM','exit':'exit_sm'})
		# smach.StateMachine.add('LAND', Land(), transitions={'exit':'exit_sm'})

		flight_sm = smach.StateMachine(outcomes=['exit_flight_sm'])

		with flight_sm:

			# smach.StateMachine.add('STANDBY', flight.Standby(), transitions={'toMISSION':'MISSION','exit_flight':'exit_flight_sm'})
			smach.StateMachine.add('MISSION', flight.Mission(), transitions={'exit_flight':'exit_flight_sm'})

		smach.StateMachine.add('FLIGHT_SM', flight_sm, transitions={'exit_flight_sm':'exit_sm'})	

	introspect = smach_ros.IntrospectionServer('server', sm, '/SM')
	introspect.start()

	#to run SMACH Viewer: rosrun smach_viewer smach_viewer.py

	outcome = sm.execute()	
	
if __name__ == '__main__':
    main()