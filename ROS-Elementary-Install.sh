#!/bin/sh

sudo apt-get -y install software-properties-common
#echo "export ROS_OS_OVERRIDE=elementary" >> ~/.bashrc
#. ~/.bashrc
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo add-apt-repository -y ppa:webupd8team/atom
sudo apt-get -y update
sudo apt-get -y install ros-kinetic-desktop-full
sudo apt-get -y install atom
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
. ~/.bashrc
sudo apt-get -y install python-rosinstall python-rosinstall-generator python-wstool build-essential ros-lunar-catkin
cd
git clone https://github.com/RoboticsClubatUCF/Laki2.git
cd Laki2/catkin_ws
catkin_make
cd
git clone git clone https://github.com/PX4/Firmware.git
cd Firmware
git submodule update --init --update
make posix_sitl_default gazebo
echo "source ~/Laki2/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "source ~/Firmware/Tools/setup_gazebo.bash ~/Firmware ~Firmware/build/posix_sitl_default" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Firmware" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Firmware/Tools/sitl_gazebo" >> ~/.bashrc
echo "export PX4_HOME_LAT=28.585586" >> ~/.bashrc
echo "export PX4_HOME_LON=-81.199212" >> ~/.bashrc
. ~/.bashrc

