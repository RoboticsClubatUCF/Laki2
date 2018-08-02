#!/usr/bin/env python

import rospy, mavros
import math, smach, smach_ros
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State, RCIn
from geometry_msgs.msg import PoseStamped, TwistStamped
from custom_msgs.msg import StateMachine

class Preflight(smach.State):

	def getMavrosState(data):

		global current_state
		current_state = data	

	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome1'])
		
	def execute(self, userdata):
		
		rate = rospy.Rate(60)
			

class Foo(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['exit','toBAR'])
		
		# rospy.init_node('preflight_node', anonymous=True)
		self.counter = 0

	def execute(self, userdata):
		rospy.loginfo('Exectuing state FOO')
		if self.counter < 3:
			self.counter += 1
			return 'outcome2'
		else:
			return 'outcome1'	

class Bar(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome1'])

	def execute(self, userdata):
		rospy.loginfo('Executing state BAR')
		return 'outcome1'		

def main():
	
	rospy.init_node('laki2_sm', anonymous=True)
	
	rospy.Subscriber("/mavros/state", State, self.getMavrosState)

	armCommandSrv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
	setModeSrv = rospy.ServiceProxy("/mavros/set_mode", SetMode) #http://wiki.ros.org/mavros/CustomModes

	sm = smach.StateMachine(outcomes=['exit_sm'])

    # Open the container
	with sm:

		smach.StateMachine.add('FOO', Foo(), transitions={'exit':'exit_sm', 'toBAR':'BAR'})
		smach.StateMachine.add('BAR', Bar(), transitions={'outcome1':'FOO'})
		smach.StateMachine.add('PREFLIGHT', Preflight(), transitions={'outcome1':'FOO'})


	outcome = sm.execute()	
	
if __name__ == '__main__':
    main()