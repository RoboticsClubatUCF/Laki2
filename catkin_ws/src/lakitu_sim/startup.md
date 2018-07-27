SETUP:

	NAVIGATE TO ~/ "git clone https://github.com/PX4/Firmware.git" to get PX4 ros package
	INSIDE FIRMWARE "git submodule update --init --update"
	THEN RUN: "make posix_sitl_default gazebo"
  
  ADD THE FOLLOWING TO ~/.bashrc
    source ~/Laki2/catkin_ws/devel/setup.bash
    source ~/Firmware/Tools/setup_gazebo.bash ~/Firmware ~Firmware/build/posix_sitl_default
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Firmware
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Firmware/Tools/sitl_gazebo


LAUNCHING SIMULATOR:
	
	roslaunch lakitu_sim [launch_file]
  
  Publishing flight targets: (MUST BE DONE FIRST)
	
		rostopic pub -l /lakitu/flight_target geometry_msgs/PoseStamped {'header: {stamp: now},pose: {position: {x: 0,y: 0,z: 5}, orientation: {x: 0,y: 0,z: 0,w: 0}'}} -r 120
  
  Flipping start switch:

		rostopic pub -l /mavros/rc/in mavros_msgs/RCIn {'channels: [0,0,0,0,0,0,2113,0,0,0,0,0,0,0,0,0,0,0]'}
	
	
SENDING COMMANDS (format):

	Setting to OFFBOARD MODE (needed for sending velocities):
			"rosservice call /mavros/set_mode 0 "OFFBOARD"" //OFFBOARD DOCUMENTATION: https://docs.px4.io/en/flight_modes/offboard.html
	
	Arming:
			"rosservice call /mavros/cmd/arming [true|false]" //where true|false is armed/isn't armed
			
			//NOTE: both arming and setting to offboard are already implemented in the code, the above is to do so from the command line
	
	Sending Velocities:	
		"rostopic pub /mavros/setpoint_vecity/cmd_vel geometry_msgs/TwistStamped {'header: {stamp: now}, twist: {linear: {x: 1, y: 0, z: 0}}'} -r 60 "
		//where -r 60 is rate in hertz
		
	ROS Topic Commands:
		rostopic list //gives all currently defined topics
    rostopic info [topic_name] //tells you everything about a topic 
		rostopic echo [topic_name] //outputs messages published to [topic_name]
