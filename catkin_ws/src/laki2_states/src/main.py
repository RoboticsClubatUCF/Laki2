#!/usr/bin/env python

import rospy, mavros
import math, smach, smach_ros
from mavros_msgs.srv import CommandBool, SetMode, ParamGet
from mavros_msgs.msg import State, RCIn
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped

''' the main states for Laki2's state machine (SMACH)'''

# initializing global variables to NONE
rcNum = None
current_state = None
current_pos = None
target = None
gotTarget = False

# callback that reads the /mavros/state topic
def getMavrosState(data):

	global current_state
	current_state = data	

# callback that reads the /mavros/rc/in topic
# /mavros/rc/in publishes an RCIn topic, which provides an array of RC channels
# provided array can be used to simulate a real-world rc controller
def getRCChannels(data):

	global rcNum
	rcNum = data.channels[6] #channels[6] is the ON/OFFswitch (2113 is ON)	

# callback that reads the /laki2/flight_target/local topic
# flight_target/local is ideally published by the path-planning system to guide Laki2 
def getLocalTarget(data):

	global target
	global gotTarget
	gotTarget = True
	target = data

# reads current position from /mavros/local_position/odom
def getCurrentPosition(data):

	global current_pos
	current_pos = data	


class Preflight(smach.State):

	'''PREFLIGHT state houses all 'preflight checks' and sets up for following states, always comes first'''

	def __init__(self):
		smach.State.__init__(self, outcomes=['toTAKEOFF','toFLIGHT'])
		
	def execute(self, userdata):

		rate = rospy.Rate(30) #runs following loop at 30hz

		while not rospy.is_shutdown():

			rate.sleep()
			
			# does nothing if mavros is not up and running yet (/mavros/state)
			if (current_state is None):
				rospy.loginfo('MAVROS NOT RUNNING')
				continue

			if (current_pos is None):
				rospy.loginfo('NO ODOMETRY DATA')
				continue	

			#does nothing if no target is published	(/laki2/flight_target)
			if(not gotTarget):
				continue	

			#listens for RC switch, if ON (2113) switches to TAKEOFF state
			#rcNum is from function getRCChannels()
			if rcNum == 2113: 
				rospy.loginfo("RC SWITCH: ON")
				return 'toTAKEOFF'



class Takeoff(smach.State):

	'''uses built-in AUTO.TAKEOFF to lift to a parameter-set altitude (2.5m by default)'''

	def __init__(self):
		smach.State.__init__(self, outcomes=['toPREFLIGHT','toFLIGHT'])

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

		self.getTakeoffAlt()
		rospy.loginfo(self.targetAlt)

		rate = rospy.Rate(30)

		while not rospy.is_shutdown():
			#checks for built-in PX4 takeoff state and if we have a flight target

			# arming Laki2, must be armed before takeoff
			if (not current_state.armed): 
				armCommandSrv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)		
				armResponse = armCommandSrv(True)
				rospy.loginfo(armResponse)

			if ((current_state.mode != 'AUTO.TAKEOFF') and (target is not None) and current_state.armed):	
				try:	#service call to set mode to takeoff
					setModeSrv = rospy.ServiceProxy("/mavros/set_mode", SetMode) #http://wiki.ros.org/mavros/CustomModes
					setModeResponse = setModeSrv(0, 'AUTO.TAKEOFF')
					# rospy.loginfo(str(setModeResponse) + '\nMODE: %s' % current_state.mode)

				except rospy.ServiceException, e:
					rospy.loginfo('Service call failed: %s' %e)

			#if we're relatively close to desired altitude, switch to flight		
			if (current_pos.pose.pose.position.z <= (self.targetAlt + float(.1)) and  current_pos.pose.pose.position.z >= (self.targetAlt - float(.1))):
				return 'toFLIGHT'

			# if (current_state.mode == 'AUTO.TAKEOFF'):
			# 	if (current_pos.pose.pose.position.z <= (self.targetAlt + .1) and  current_pos.pose.pose.position.z >= (self.targetAlt - .1)):
			# 		return 'toFLIGHT'	

			rate.sleep()	

#PART OF FLIGHT_SM
class Standby(smach.State):
	
	def __init__(self):
		smach.State.__init__(self, outcomes=['toLOCAL','exit'])

	def execute(self, userdata):
	
		if ((current_state.mode != 'AUTO.LOITER')):	
			try:	#service call to set mode to takeoff
				setModeSrv = rospy.ServiceProxy("/mavros/set_mode", SetMode) #http://wiki.ros.org/mavros/CustomModes
				setModeResponse = setModeSrv(0, 'AUTO.LOITER')
				# rospy.loginfo(str(setModeResponse) + '\nMODE: %s' % current_state.mode)

			except rospy.ServiceException, e:
				rospy.loginfo('Service call failed: %s' %e)	


#PART OF FLIGHT_SM
class ToLocal(smach.State):


	'''A flight mode that allows navigation to local setpoints'''

	def __init__(self):
		smach.State.__init__(self, outcomes=['toSTANDBY','exit'])
		self.target_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=100)
		# self.target_pub = None
		
	def publishSetpoint(self,target):
			
		self.target_pub.publish(target)

	def execute(self, userdata):
		
		rospy.loginfo("FLIGHT REACHED")

		rate = rospy.Rate(30)

		rospy.Subscriber("/laki2/flight_target/local", PoseStamped, self.publishSetpoint) #tells where we're going

		while not rospy.is_shutdown():

			if ((current_state.mode != 'OFFBOARD') and (gotTarget) and current_state.armed):	
				try:	#service call to set mode to OFFBOARD
					setModeSrv = rospy.ServiceProxy("/mavros/set_mode", SetMode) #http://wiki.ros.org/mavros/CustomModes
					setModeResponse = setModeSrv(0, 'OFFBOARD')
					# rospy.loginfo(str(setModeResponse) + '\nMODE: %s' % current_state.mode)

				except rospy.ServiceException, e:
					rospy.loginfo('Service call failed: %s' %e)

						

			rate.sleep()


def main():
	
	rospy.init_node('laki2_sm', anonymous=True)
	
	rospy.Subscriber("/mavros/state", State, getMavrosState) #provides mavros state (connected, flight mode, etc.)
	rospy.Subscriber("/mavros/rc/in", RCIn, getRCChannels) #subscribe to mavros topic for RC Data
	rospy.Subscriber("/laki2/flight_target/local", PoseStamped, getLocalTarget) #tells where we're going
	rospy.Subscriber('/mavros/local_position/odom', Odometry, getCurrentPosition) #gives current, continuous position

	sm = smach.StateMachine(outcomes=['exit_sm'])

    # Open the container, declare the SMACH states
	with sm:

		smach.StateMachine.add('PREFLIGHT', Preflight(), transitions={'toTAKEOFF': 'TAKEOFF','toFLIGHT':'FLIGHT_SM'})
		smach.StateMachine.add('TAKEOFF', Takeoff(), transitions={'toPREFLIGHT': 'PREFLIGHT','toFLIGHT': 'FLIGHT_SM'})
		# smach.StateMachine.add('FLIGHT', Flight(), transitions={'toPREFLIGHT':'PREFLIGHT','exit':'exit_sm'})

		flight_sm = smach.StateMachine(outcomes=['exit_flight_sm'])

		with flight_sm:

			smach.StateMachine.add('TO_LOCAL', ToLocal(), transitions={'toSTANDBY':'STANDBY','exit':'exit_flight_sm'})
			smach.StateMachine.add('STANDBY', Standby(), transitions={'toTo_LOCAL':'TO_LOCAL','exit':'exit_flight_sm'}

		smach.StateMachine.add('FLIGHT_SM', flight_sm, transitions={'exit_flight_sm':'exit_sm'})	

	introspect = smach_ros.IntrospectionServer('server', sm, '/SM')
	introspect.start()

	outcome = sm.execute()	
	
if __name__ == '__main__':
    main()