#!/usr/bin/env python

import cmd, sys
import rospy
from mavros_msgs.srv import ParamSet, ParamGet

class Value():

	def __init__(self, integer, real):
		self.integer = integer
		self.real = real

class SimShell(cmd.Cmd):

	intro = 'Testing shell for Laki2_sim'
	prompt = '(Laki2)'
	file = None
	# setParamSrv = rospy.ServiceProxy('/mavros/param/set', ParamSet)

	def do_kill(self, arg):
		'Kills sensor named by arg'
		
		if arg == 'rc':
			param = 'SIM_RC_FAIL'
			value = Value(1, 0)
		elif arg == 'gps':
			param = 'SIM_GPS_DISABLE'
			value = Value(1,0)	
		set_param(param, value)

	def do_revive(self, arg):	

		if arg == 'rc':
			param = 'SIM_RC_FAIL'
			value = Value(0, 0)
		elif arg == 'gps':
			param = 'SIM_GPS_DISABLE'
			value = Value(0,0)	
		set_param(param, value)	

	def do_end(self, arg):
		return True	

def set_param(param, value):
	
	setParamSrv = rospy.ServiceProxy('/mavros/param/set', ParamSet)

	try:
		response = setParamSrv(param, value)
		print(response)	
	except rospy.ServiceException, e:
		rospy.loginfo('Service call failed: %s' %e)		

if __name__=='__main__':

	rospy.init_node('sim_shell', anonymous=True)
	SimShell().cmdloop('Interactive testing suite for Laki2_sim')
