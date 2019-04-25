#!/usr/bin/env python

import rospy, mavros
import math, smach, smach_ros
from mavros_msgs.srv import CommandBool, SetMode, ParamGet, CommandTOL
from mavros_msgs.msg import State, RCIn
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix

from laki2_common import TextColors

import flight, monitor

''' the main states for Laki2's state machine (SMACH)'''

# initializing global variables to NONE
rcChannels = None
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

	global rcChannels
	rcChannels = data.channels #channels[4] == 'g' switch on RC controller (2067|961 -> ON|OFF)

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

	
# SM State: Standby
# From: 	Any other state
# Purpose:	Does nothing, allows FCU and RC to run the show
#			Waits for RC switch to switch back to preflight
class Standby(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['exit','toPREFLIGHT','toFLIGHT_SM'])

		self.TAG = "STANDBY"

	def execute(self, userdata):
	
		rate = rospy.Rate(30)

		while not rospy.is_shutdown():

			rate.sleep()

			if rcChannels[4] == 2067:
				return 'toPREFLIGHT'		


# SM State: PREFLIGHT
# From: 	NONE
# Purpose: 	basic preflight checks (working odometry, connection to FCU)
#			listens for start switch to begin flight			
class Preflight(smach.State):

	'''PREFLIGHT state houses all 'preflight checks' and sets up for following states, always comes first'''

	def __init__(self):
		# smach.State.__init__(self, outcomes=['toTAKEOFF','toFLIGHT'])
		smach.State.__init__(self, outcomes=['exit','toTAKEOFF','toSTANDBY'])

		self.TAG = "PREFLIGHT"
		
	def execute(self, userdata):

		rate = rospy.Rate(30) #runs following loop at 30hz

		#flags to stop constant output of warning messages
		targetFlag = True
		readyFlag = True
		noOdomFlag = True
		testFlag = False

		while not rospy.is_shutdown():

			rate.sleep()

			if self.preempt_requested():
				self.service_preempt()
				return 'toSTANDBY'
			
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
			# rcChannelsis from function getRCChannels()
			if (rcChannels[4] == 2067): 
				rospy.loginfo(TextColors.OKGREEN + "RC SWITCH: ON" + TextColors.ENDC)
				return 'toTAKEOFF'

			# if testFlag:	
				


# SM State: TAKEOFF
# From: 	PREFLIGHT
# Purpose:	to takeoff from the ground to a hardcoded height
class Takeoff(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['exit','toPREFLIGHT','toFLIGHT_SM','toSTANDBY'])

		self.TAG = "TAKEOFF"

	def execute(self, userdata):

		rate = rospy.Rate(30)

		if(current_state.mode == "GUIDED"):
			guided = True
		else:
			guided = False	

		takeoffFlag = True	

		while not rospy.is_shutdown():

			rate.sleep()

			# 961 is up on switch 'g'
			# if (rcNum == 961):

			# 	if(current_state.mode != 'STABILIZE'):
			# 		setMode("STABILIZE")

			# continue

			if self.preempt_requested():
				self.service_preempt()
				return 'toSTANDBY'

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
				takeoffFlag = False

				if takeoffResponse.success == True:

					while not (current_pos.pose.pose.position.z <= 1.1 and current_pos.pose.pose.position.z >= 0.9):
						rate.sleep()
						
					return 'toFLIGHT_SM'


# adapted from RoboSub 
class Safety(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['PREEMPTED', 'toSTANDBY'])

		self.TAG = "SAFETY"
		self.rcStop = False
		self.gpsError = False

		rospy.Subscriber('/mavros/rc/in', RCIn, self.radioCB)
		rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gpsCB)
		
	def radioCB(self, data):

		if data.channels[4] == 961:
			self.rcStop = True

	def	gpsCB(self, data):

		if data.status == -1:
			self.gpsError = True

	def execute(self, userdata):

		rate = rospy.Rate(30)

		while not self.rcStop and not self.gpsError and not self.preempt_requested():
			rate.sleep()

		if self.preempt_requested():
			self.service_preempt()
			return 'PREEMPTED'
		elif self.rcStop:
			self.rcStop = False
			# if current_state.mode != "STABILIZE":
			# 	setMode('STABILIZE')
			return 'toSTANDBY'
		elif self.gpsError:
			self.gpsError = False
			# if current_state.mode != "LOITER":
			# 	setMode('LOITER')
			return 'toSTANDBY'

# shamelessly lifted from RoboSub
def safetyWrap(task):		

	def safety_outcome(outcome_map):
		rospy.logerr(outcome_map)
		if outcome_map['SAFETY'] != 'PREEMPTED':
			return outcome_map['SAFETY']
		elif outcome_map[task.TAG] is not None:
			return outcome_map[task.TAG]
		else:
			return 'ABORT'

	def safety_term(outcome_map):
		return True

	safetyState = Safety()
	outcomes = list(task.get_registered_outcomes()) + list(safetyState.get_registered_outcomes())
	outcomes[:] = [x for x in outcomes if x != 'PREEMPTED']

	sm_wrapper = smach.Concurrence(outcomes,
					default_outcome='toSTANDBY',
					outcome_cb=safety_outcome,
					child_termination_cb=safety_term)

	with sm_wrapper:
		smach.Concurrence.add("SAFETY", Safety())
		smach.Concurrence.add(task.TAG, task)

	return sm_wrapper


def main():
	
	rospy.init_node('laki2_sm', anonymous=True)

	rospy.Subscriber('/mavros/state', State, getMavrosState) #provides mavros state (connected, flight mode, etc.)
	rospy.Subscriber('/mavros/rc/in', RCIn, getRCChannels) #subscribe to mavros topic for RC Data
	rospy.Subscriber('/mavros/local_position/odom', Odometry, getCurrentPosition) #gives current, continuous position (odom frame)

	sm = smach.StateMachine(outcomes=['exit_sm'])
	# takeoff_sm = smach.StateMachine(outcomes=['to_flight_sm', 'preempt','exit','toSTANDBY'])
	# flight_sm = smach.StateMachine(outcomes=['exit_to_preflight','toSTANDBY','exit_sm', 'exit_flight_sm'])

    # Open the container, declare the SMACH states
	with sm:

		smach.StateMachine.add('PREFLIGHT', safetyWrap(Preflight()), transitions={'exit':'exit_sm', 'toTAKEOFF':'TAKEOFF', 'toSTANDBY':'STANDBY'})

		smach.StateMachine.add('TAKEOFF', safetyWrap(Takeoff()), transitions={'exit':'exit_sm','toPREFLIGHT':'PREFLIGHT', 'toFLIGHT_SM':'FLIGHT_SM', 'toSTANDBY':'STANDBY'})

		smach.StateMachine.add('FLIGHT_SM', flight.makeTask(), transitions={'exit_toPREFLIGHT':'PREFLIGHT', 'exit_toSTANDBY':'STANDBY', 'DONE':'exit_sm'})

		smach.StateMachine.add('STANDBY', Standby(), transitions={'exit':'exit_sm','toPREFLIGHT':'PREFLIGHT', 'toFLIGHT_SM':'FLIGHT_SM'})
		
	introspect = smach_ros.IntrospectionServer('server', sm, '/SM')
	introspect.start()

	#to run SMACH Viewer: rosrun smach_viewer smach_viewer.py

	outcome = sm.execute()	
	
if __name__ == '__main__': 
    main() 