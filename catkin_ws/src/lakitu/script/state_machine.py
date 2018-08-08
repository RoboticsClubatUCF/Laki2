#!/usr/bin/env python

import rospy, mavros
import math, smach, smach_ros
from mavros_msgs.srv import CommandBool, SetMode, ParamGet
from mavros_msgs.msg import State, RCIn
from geometry_msgs.msg import PoseStamped, TwistStamped

# initializing global variables to NONE
rcNum = None
current_state = None
target = None

# callback that reads the /mavros/state topic
def getMavrosState(data):

	global current_state
	current_state = data	

# callback that reads the /mavros/rc/in topic
def getRCChannels(data):

	global rcNum
	rcNum = data.channels[6]	

# callback that reads the /lakitu/flight_target topic 
def getFlightTarget(data):

	global target
	target = data


class Preflight(smach.State):

	'''PREFLIGHT state houses all 'preflight checks' and sets up for following states, always comes first'''

	def __init__(self):
		smach.State.__init__(self, outcomes=['toTAKEOFF','toFLIGHT'])
		
	def execute(self, userdata):
		
		rate = rospy.Rate(60)

		while not rospy.is_shutdown():
			
			# does nothing if mavros is not up and running yet (/mavros/state)
			if (current_state is None):
				rospy.loginfo('MAVROS NOT RUNNING')
				continue

			#does nothing if no target is published	(/lakitu/flight_target)
			if(target is None):
				continue	

			#listens for RC switch, if ON (2113) switches to TAKEOFF state
			#rcNum is from function getRCChannels()
			if rcNum == 2113: 
				rospy.loginfo("RC SWITCH: ON")
				return 'toTAKEOFF'

			rate.sleep() # sleep for 1/60th of a second


class Takeoff(smach.State):

	'''dont use offboard mode for takeoff, moron'''

	def __init__(self):
		smach.State.__init__(self, outcomes=['toPREFLIGHT','toFLIGHT'])

	def getTakeoffAlt(self):

		'''reading the PX4 param 'MIS_TAKEOFF_ALT'
		useful method to read any PX4 param in script
		https://docs.px4.io/en/advanced_config/parameter_reference.html'''

		try:
			takeoffAltSrv = rospy.ServiceProxy('/mavros/param/get', ParamGet)	
			self.targetAlt = takeoffAltSrv('MIS_TAKEOFF_ALT')

		except rospy.ServiceException, e:
			rospy.loginfo('Service call failed: %s' %e)	

	def execute(self, userdate):

		self.getTakeoffAlt()
		rospy.loginfo(self.targetAlt)
		
		#checks for built-in PX4 takeoff state and if we have a flight target
		if ((current_state.mode != 'AUTO.TAKEOFF') and (target is not None)):	
			try:	#service call to set mode to takeoff
				setModeSrv = rospy.ServiceProxy("/mavros/set_mode", SetMode) #http://wiki.ros.org/mavros/CustomModes
				setModeResponse = setModeSrv(0, 'AUTO.TAKEOFF')
				rospy.loginfo(str(setModeResponse) + '\nMODE: %s' % current_state.mode)

			except rospy.ServiceException, e:
				rospy.loginfo('Service call failed: %s' %e)

		if (not current_state.armed):
			armCommandSrv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)		
			armResponse = armCommandSrv(True)
			rospy.loginfo(armResponse)

		if (current_state.mode == 'AUTO.TAKEOFF'):
			return 'toFLIGHT'
		else:
			return 'toPREFLIGHT'		


class Flight(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['toPREFLIGHT'])

	def execute(self, userdata):
		# pass
		rospy.loginfo("FLIGHT REACHED")
		return 'toPREFLIGHT'	

def main():
	
	rospy.init_node('laki2_sm', anonymous=True)
	
	rospy.Subscriber("/mavros/state", State, getMavrosState) #provides mavros state (connected, flight mode, etc.)
	rospy.Subscriber("/mavros/rc/in", RCIn, getRCChannels) #subscribe to mavros topic for RC Data
	rospy.Subscriber("/lakitu/flight_target", PoseStamped, getFlightTarget) #tells where we're going

	sm = smach.StateMachine(outcomes=['exit_sm'])

    # Open the container, declare the SMACH states
	with sm:

		smach.StateMachine.add('PREFLIGHT', Preflight(), transitions={'toTAKEOFF': 'TAKEOFF','toFLIGHT':'FLIGHT'})
		smach.StateMachine.add('TAKEOFF', Takeoff(), transitions={'toPREFLIGHT': 'PREFLIGHT','toFLIGHT': 'FLIGHT'})
		smach.StateMachine.add('FLIGHT', Flight(), transitions={'toPREFLIGHT':'PREFLIGHT'})

	outcome = sm.execute()	
	
if __name__ == '__main__':
    main()