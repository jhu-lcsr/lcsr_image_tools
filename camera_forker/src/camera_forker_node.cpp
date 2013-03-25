
#include <ros/ros.h>

#include <camera_forker/camera_forker.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "camera_forker");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  camera_forker::CameraForker forker(nh, nh_private);

  ros::spin();

  return 0;
}
