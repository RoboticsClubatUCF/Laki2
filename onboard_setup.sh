
#!/bin/sh

sudo apt-get -y install software-properties-common
#echo "export ROS_OS_OVERRIDE=elementary" >> ~/.bashrc
#. ~/.bashrc
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo add-apt-repository -y ppa:webupd8team/atom
sudo apt-get -y update
sudo apt-get -y install ros-kinetic-desktop-full
sudo apt-get -y install ros-kinetic-mavros ros-kinetic-mavros-extras
# sudo apt-get -y install atom
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
. ~/.bashrc
sudo apt-get -y install python-rosinstall python-rosinstall-generator python-wstool build-essential ros-lunar-catkin
cd
# git clone https://github.com/RoboticsClubatUCF/Laki2.git

cd

sudo apt-get -y install python-jinja2
sudo apt-get -y install python-pip python-dev
sudo pip install numpy toml
# sudo pip install opencv-python
# sudo apt-get -y install python-opencv
# sudo apt-get -y install git

cd ~/Laki2/catkin_ws/src/laki2_common 
sudo pip install .

echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
. ~/.bashrc
