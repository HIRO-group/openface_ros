cmake_minimum_required(VERSION 2.8.3)
project(openface_ros)

add_compile_options(-std=c++14)
add_compile_options(-fPIC)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES openface_ros
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}/boost
)

add_library(openface_ros STATIC include/openface_ros/openface_ros.h
                                src/openface_ros.cpp)
add_dependencies(openface_ros         
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(openface_ros
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(openface_ros LandmarkDetector boost_filesystem boost_system dlib openblas)
target_link_libraries(openface_ros FaceAnalyser)
target_link_libraries(openface_ros GazeAnalyser)
target_link_libraries(openface_ros Utilities)

add_executable(openface_realsense src/openface_realsense.cpp)
add_dependencies(openface_realsense         
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(openface_realsense openface_ros)
target_link_libraries(openface_realsense
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(openface_realsense LandmarkDetector boost_filesystem boost_system dlib openblas)
target_link_libraries(openface_realsense FaceAnalyser)
target_link_libraries(openface_realsense GazeAnalyser)
target_link_libraries(openface_realsense Utilities)
