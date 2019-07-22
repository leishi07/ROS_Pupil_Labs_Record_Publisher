cmake_minimum_required(VERSION 2.8.3)
project(record_pub)

add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(record_pub src/record_pub.cpp )
target_link_libraries(record_pub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
