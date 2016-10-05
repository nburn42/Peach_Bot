source /opt/ros/jade/setup.bash
source catkin_ws/devel/setup.sh
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/jade/lib/arm-linux-gnueabihf/
sudo umount /dev/mmcblk1p1
sudo mount /dev/mmcblk1p1 /mnt
