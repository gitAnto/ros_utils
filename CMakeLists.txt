cmake_minimum_required(VERSION 2.8.3)
project(ros_utils)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs geometry_msgs genmsg cv_bridge image_transport)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(rgbd_throttle src/rgbd_throttle.cpp)
target_link_libraries(rgbd_throttle ${catkin_LIBRARIES})
add_dependencies(rgbd_throttle rgbd_throttle_generate_messages_cpp)

add_executable(pose_noise_generator src/pose_noise_generator.cpp)
target_link_libraries(pose_noise_generator ${catkin_LIBRARIES})
add_dependencies(pose_noise_generator pose_noise_generator_generate_messages_cpp)

add_executable(image_encoding_converter src/image_encoding_converter.cpp)
target_link_libraries(image_encoding_converter ${catkin_LIBRARIES})
add_dependencies(image_encoding_converter image_encoding_converter_generate_messages_cpp)
