cmake_minimum_required(VERSION 2.8.3)
project(yaws)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(yaws src/yaws.cpp)
target_link_libraries(yaws ${catkin_LIBRARIES})
add_dependencies(yaws yaws_generate_messages_cpp)

add_executable(yaws_listener src/yaws_listener.cpp)
target_link_libraries(yaws_listener ${catkin_LIBRARIES})
add_dependencies(yaws_listener yaws_generate_messages_cpp)

add_executable(yaws_controller src/yaws_controller.cpp)
target_link_libraries(yaws_controller ${catkin_LIBRARIES})
add_dependencies(yaws_controller yaws_generate_messages_cpp)
