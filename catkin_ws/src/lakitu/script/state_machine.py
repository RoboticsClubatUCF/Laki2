#!/usr/bin/env python

import rospy, mavros
import math, smach, smach_ros
from mavros_msgs.srv import CommandBool, SetMode, ParamGet
from mavros_msgs.msg import State, RCIn
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped

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
def getRCChannels(data):

	global rcNum
	rcNum = data.channels[6]	

# callback that reads the /lakitu/flight_target topic 
def getFlightTarget(data):

	global target
	global gotTarget
	gotTarget = True
	target = data

def getCurrentPosition(data):

	global current_pos
	current_pos = data	


class Preflight(smach.State):

	'''PREFLIGHT state houses all 'preflight checks' and sets up for following states, always comes first'''

	def __init__(self):
		smach.State.__init__(self, outcomes=['toTAKEOFF','toFLIGHT'])
		
	def execute(self, userdata):

		rate = rospy.Rate(30)

		while not rospy.is_shutdown():

			rate.sleep()
			
			# does nothing if mavros is not up and running yet (/mavros/state)
			if (current_state is None):
				rospy.loginfo('MAVROS NOT RUNNING')
				continue

			if (current_pos is None):
				rospy.loginfo('NO ODOMETRY DATA')
				continue	

			#does nothing if no target is published	(/lakitu/flight_target)
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


			if (current_pos.pose.pose.position.z <= (self.targetAlt + float(.1)) and  current_pos.pose.pose.position.z >= (self.targetAlt - float(.1))):
				return 'toFLIGHT'

			# if (current_state.mode == 'AUTO.TAKEOFF'):
			# 	if (current_pos.pose.pose.position.z <= (self.targetAlt + .1) and  current_pos.pose.pose.position.z >= (self.targetAlt - .1)):
			# 		return 'toFLIGHT'	

			rate.sleep()		


class Flight(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['toPREFLIGHT','exit'])
		self.target_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=100)
		# self.target_pub = None
		
	def setpoint_flight(self,target):
			
		self.target_pub.publish(target)

	def execute(self, userdata):
		
		rospy.loginfo("FLIGHT REACHED")

		rate = rospy.Rate(30)

		rospy.Subscriber("/lakitu/flight_target", PoseStamped, self.setpoint_flight) #tells where we're going

		

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
	rospy.Subscriber("/lakitu/flight_target", PoseStamped, getFlightTarget) #tells where we're going
	rospy.Subscriber('/mavros/local_position/odom', Odometry, getCurrentPosition) #gives current, continuous position

	sm = smach.StateMachine(outcomes=['exit_sm'])

    # Open the container, declare the SMACH states
	with sm:

		smach.StateMachine.add('PREFLIGHT', Preflight(), transitions={'toTAKEOFF': 'TAKEOFF','toFLIGHT':'FLIGHT'})
		smach.StateMachine.add('TAKEOFF', Takeoff(), transitions={'toPREFLIGHT': 'PREFLIGHT','toFLIGHT': 'FLIGHT'})
		smach.StateMachine.add('FLIGHT', Flight(), transitions={'toPREFLIGHT':'PREFLIGHT','exit':'exit_sm'})

	introspec = smach_ros.IntrospectionServer('server', sm, '/SM')
	introspec.start()

	outcome = sm.execute()	
	
if __name__ == '__main__':
    main()