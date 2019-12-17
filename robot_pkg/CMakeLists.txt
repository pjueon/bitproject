cmake_minimum_required(VERSION 2.8.3)
project(robot_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  bit_custom_msgs
  tf
)

link_directories(/usr/local/lib)

set(OPENCV_LIBS opencv_core opencv_imgcodecs opencv_highgui opencv_imgproc opencv_calib3d opencv_features2d)
set(OPENCV_INCLUDE_DIR /usr/local/include/opencv4)

# C++17 support
set(CMAKE_CXX_FLAGS "-std=c++1z ${CMAKE_CXX_FLAGS}")

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIR} 
)

file(GLOB SOURCES "src/*.cpp")

add_executable(robot_node ${SOURCES})
add_dependencies(robot_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(robot_node
   ${catkin_LIBRARIES} 
   ${OPENCV_LIBS}
)

