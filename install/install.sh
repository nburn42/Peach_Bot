#http://wiki.ros.org/indigo/Installation/UbuntuARM
sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update

sudo apt-get install -y ros-jade-ros-base ca-certificates
sudo c_rehash /etc/ssl/certs
sudo apt-get install -y python-rosdep python-pygame
sudo rosdep init
rosdep update
sudo apt-get install -y python-rosinstall
sudo apt-get install -y ros-jade-ecl-threads
sudo apt-get install -y ros-jade-opencv3
#cd ~/Peach_Bot/install
#./ZED_SDK_Linux_JTX1_v1.0.0c.run

#cd ~/Peach_Bot/catkin_ws/src
#git clone https://github.com/stereolabs/zed-ros-wrapper
