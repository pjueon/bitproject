cmake_minimum_required(VERSION 2.8.3)
project(vtree_train)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
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

add_executable(${PROJECT_NAME} ${SOURCES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES} 
   ${OPENCV_LIBS}
)

