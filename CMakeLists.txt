cmake_minimum_required(VERSION 2.8.3)
project(rgbd_throttle)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs genmsg)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(rgbd_throttle src/rgbd_throttle.cpp)
target_link_libraries(rgbd_throttle ${catkin_LIBRARIES})
add_dependencies(rgbd_throttle rgbd_throttle_generate_messages_cpp)
