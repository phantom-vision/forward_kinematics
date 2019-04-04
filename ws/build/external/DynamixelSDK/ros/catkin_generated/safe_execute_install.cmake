execute_process(COMMAND "/home/sachin/Desktop/ws/forward_kinematics/ws/build/external/DynamixelSDK/ros/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/sachin/Desktop/ws/forward_kinematics/ws/build/external/DynamixelSDK/ros/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
