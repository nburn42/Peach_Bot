execute_process(COMMAND "/home/ubuntu/Peach_Bot/catkin_ws/build_isolated/rosunit/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/ubuntu/Peach_Bot/catkin_ws/build_isolated/rosunit/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
