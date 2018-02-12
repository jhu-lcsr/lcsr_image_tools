cmake_minimum_required(VERSION 2.8.3)
project(fakecam)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS rospy roscpp image_transport
  camera_calibration_parsers tf resource_retriever terse_roscpp dynamic_reconfigure
  cv_bridge)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

generate_dynamic_reconfigure_options(
  cfg/FakeCam.cfg
)

# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
  DEPENDS rospy roscpp image_transport camera_calibration_parsers tf
  resource_retriever terse_roscpp cv_bridge
  CATKIN_DEPENDS # TODO
  INCLUDE_DIRS 
  LIBRARIES 
  )

include_directories(${catkin_INCLUDE_DIRS})

add_executable(fakecam_node src/fakecam_node.cpp)
add_dependencies(fakecam_node ${PROJECT_NAME}_gencfg)
target_link_libraries(fakecam_node ${catkin_LIBRARIES})

add_executable(video_file_publisher src/video_file_publisher.cpp)
target_link_libraries(video_file_publisher ${catkin_LIBRARIES} ${OpenCV_LIBS})

