# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(homework4)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS  tf laser_geometry roscpp rospy std_msgs genmsg)
find_package( OpenCV REQUIRED )


## Declare ROS messages and services
##add_message_files(FILES Num.msg)
##add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(  ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} )

add_executable(find_near_obstacle src/find_near_obstacle.cpp)
target_link_libraries(find_near_obstacle ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(find_near_obstacle homework4_generate_messages_cpp)

include_directories(${EIGEN_INCLUDE_DIRS})
link_directories(${EIGEN_LIBRARY_DIRS})
# %EndTag(FULLTEXT)%
