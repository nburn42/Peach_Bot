# follow directions here
#http://myzharbot.robot-home.it/blog/software/ros-nvidia-jetson-tx1-jetson-tk1-opencv-ultimate-guide/

#
#sudo cp /opt/ros/jade/lib/pkgconfig/cv_bridge.pc /opt/ros/jade/lib/pkgconfig/cv_bridge.pc-bak
#  sudo cp /opt/ros/jade/lib/pkgconfig/image_geometry.pc /opt/ros/jade/lib/pkgconfig/image_geometry.pc-bak
#  sudo cp /opt/ros/jade/share/cv_bridge/cmake/cv_bridgeConfig.cmake /opt/ros/jade/share/cv_bridge/cmake/cv_bridgeConfig.cmake-bak
#  sudo cp /opt/ros/jade/share/image_geometry/cmake/image_geometryConfig.cmake /opt/ros/jade/share/image_geometry/cmake/image_geometryConfig.cmake-bak
#  sudo gedit /opt/ros/jade/lib/pkgconfig/cv_bridge.pc &
#  sudo gedit /opt/ros/jade/lib/pkgconfig/image_geometry.pc &
#  sudo gedit /opt/ros/jade/share/cv_bridge/cmake/cv_bridgeConfig.cmake &
#  sudo gedit /opt/ros/jade/share/image_geometry/cmake/image_geometryConfig.cmake &


sudo sed -i.bak s/arm-linux-gnueabihf\\///g /opt/ros/jade/lib/pkgconfig/cv_bridge.pc
sudo sed -i.bak s/arm-linux-gnueabihf\\///g /opt/ros/jade/lib/pkgconfig/image_geometry.pc
sudo sed -i.bak s/arm-linux-gnueabihf\\///g  /opt/ros/jade/share/cv_bridge/cmake/cv_bridgeConfig.cmake
sudo sed -i.bak s/arm-linux-gnueabihf\\///g  /opt/ros/jade/share/image_geometry/cmake/image_geometryConfig.cmake

sudo sed -i.bak s/2.4.8/2.4.12/g /opt/ros/jade/lib/pkgconfig/cv_bridge.pc
sudo sed -i.bak s/2.4.8/2.4.12/g /opt/ros/jade/lib/pkgconfig/image_geometry.pc
sudo sed -i.bak s/2.4.8/2.4.12/g  /opt/ros/jade/share/cv_bridge/cmake/cv_bridgeConfig.cmake
sudo sed -i.bak s/2.4.8/2.4.12/g  /opt/ros/jade/share/image_geometry/cmake/image_geometryConfig.cmake

sudo sed -i.bak s/-l:\\/usr\\/lib\\/libopencv_ocl.so.2.4.12//g /opt/ros/jade/lib/pkgconfig/cv_bridge.pc
sudo sed -i.bak s/-l:\\/usr\\/lib\\/libopencv_ocl.so.2.4.12//g /opt/ros/jade/lib/pkgconfig/image_geometry.pc
sudo sed -i.bak s/-l:\\/usr\\/lib\\/libopencv_ocl.so.2.4.12//g  /opt/ros/jade/share/cv_bridge/cmake/cv_bridgeConfig.cmake
sudo sed -i.bak s/-l:\\/usr\\/lib\\/libopencv_ocl.so.2.4.12//g  /opt/ros/jade/share/cv_bridge/cmake/cv_bridgeConfig.cmake
sudo sed -i.bak s/-l:\\/usr\\/lib\\/libopencv_ocl.so.2.4.12//g  /opt/ros/jade/share/image_geometry/cmake/image_geometryConfig.cmake
sudo sed -i.bak s/-l:\\/usr\\/lib\\/libopencv_ocl.so.2.4.12//g  /opt/ros/jade/share/image_geometry/cmake/image_geometryConfig.cmake
