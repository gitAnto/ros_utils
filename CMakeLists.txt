cmake_minimum_required(VERSION 2.8.3)
project(ros_utils)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs genmsg)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ros_utils src/rgbd_throttle.cpp)
target_link_libraries(ros_utils ${catkin_LIBRARIES})
add_dependencies(ros_utils ros_utils_generate_messages_cpp)
