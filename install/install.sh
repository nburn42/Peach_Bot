#http://wiki.ros.org/indigo/Installation/UbuntuARM
sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get install ros-jade-ros-base
sudo apt-get install python-rosdep
sudo rosdep init
rosdep update
sudo apt-get install python-rosinstall
sudo apt-get install ros-jade-ecl-threads
