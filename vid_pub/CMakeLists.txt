cmake_minimum_required(VERSION 2.8.3)
project(vid_pub)

find_package(catkin REQUIRED COMPONENTS rospy roscpp
  dynamic_reconfigure image_transport
  camera_info_manager camera_calibration_parsers tf
  resource_retriever terse_roscpp cv_bridge)

find_package(OpenCV REQUIRED )

include_directories(include cfg/cpp ${Boost_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

#add dynamic reconfigure api
#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/VidPub.cfg)

add_executable(vid_pub_node src/vid_pub.cpp)
add_dependencies(vid_pub_node ${PROJECT_NAME}_gencfg) 
target_Link_libraries(vid_pub_node ${catkin_LIBRARIES})

# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS rospy roscpp dynamic_reconfigure image_transport camera_info_manager camera_calibration_parsers tf resource_retriever terse_roscpp opencv2 cv_bridge
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS cfg/cpp
)
