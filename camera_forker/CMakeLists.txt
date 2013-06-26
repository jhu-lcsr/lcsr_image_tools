cmake_minimum_required(VERSION 2.8.3)
project(camera_forker)

find_package(catkin REQUIRED COMPONENTS  roscpp
  image_transport camera_info_manager
  camera_calibration_parsers tf
  resource_retriever terse_roscpp cv_bridge)

# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS roscpp
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  )

include_directories(include ${catkin_INCLUDE_DIRS})

#common commands for building c++ executables and libraries
add_library(${PROJECT_NAME} src/camera_forker.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(camera_forker_node src/camera_forker_node.cpp)
target_link_libraries(camera_forker_node ${PROJECT_NAME} ${catkin_LIBRARIES})

