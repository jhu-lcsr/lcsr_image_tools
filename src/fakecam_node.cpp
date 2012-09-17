
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <camera_calibration_parsers/parse_ini.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <resource_retriever/retriever.h>
#include <dynamic_reconfigure/server.h>
#include <fakecam/FakeCamConfig.h>

class FakeCamParameters {
private:
  dynamic_reconfigure::Server<fakecam::FakeCamConfig> server_;

public:
  std::string base_frame_id;
  double fov;
  double separation;
  int convergence_method;
  double convergence_angle;
  double convergence_distance;
  std::string convergence_frame_id;

  void reconfigure_cb(fakecam::FakeCamConfig &config, uint32_t level) {
    base_frame_id = config.base_frame_id;
    fov = config.fov;
    separation = config.separation;
    convergence_method = config.convergence_method;
    convergence_angle = config.convergence_angle;
    convergence_frame_id = config.convergence_frame_id;
  }

  FakeCamParameters() {
    server_.setCallback(boost::bind(&FakeCamParameters::reconfigure_cb, this, _1, _2));
  }
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "fakecam_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Construct dynamic reconfigure server / structure
  FakeCamParameters parameters;

  // We could probably do something with the camera name, check
  // errors or something, but at the moment, we don't care.
  std::string calibration_file_uri;
  std::string camera_name;
  sensor_msgs::CameraInfo camera_info;

  nh_private.getParam("calibration_file_uri",calibration_file_uri);
  nh_private.getParam("camera_name",camera_name);

  resource_retriever::Retriever retriever;
  resource_retriever::MemoryResource resource;

  try {
    resource = retriever.get(calibration_file_uri); 
  } catch (resource_retriever::Exception& e) {
    ROS_FATAL_STREAM("Failed to retrieve camere calibration file from \""<<calibration_file_uri<<"\" error: "<<e.what());
    return -1;
  }

  if (camera_calibration_parsers::parseCalibrationIni(
        std::string((const char*)resource.data.get(),resource.size),
        camera_name,
        camera_info))
  {
    ROS_INFO_STREAM("Successfully parsed camera calibration from \""<<calibration_file_uri<<"\"");
  } else {
    ROS_FATAL_STREAM("Could not parse camera calibration from \""<<calibration_file_uri<<"\"");
    return -1;
  }

  // create image transport and publishers
  image_transport::ImageTransport it(nh_private);
  image_transport::CameraPublisher pub_left = it.advertiseCamera("fake_image_left", 1);
  image_transport::CameraPublisher pub_right = it.advertiseCamera("fake_image_right", 1);

  // Create TF broadcaster
  tf::TransformBroadcaster broadcaster;
  tf::Transform left_frame, right_frame;

  // The image never changes
  sensor_msgs::Image msg;
  msg.width = 4; 
  msg.height = 3;
  msg.encoding = "rgb8";
  msg.is_bigendian = false;
  msg.step = msg.width*3;
  msg.data.resize(msg.height*msg.step);
  for(unsigned int i=0; i<msg.data.size(); i++) {
    msg.data[i] = 0;
  }

  ros::Rate rate(10.0);

  ROS_INFO_STREAM("Publishing fake cameras...");

  tf::TransformListener listener;
  tf::StampedTransform convergence_frame;
  double convergence_angle = 0.0;

  while(ros::ok()) {
    switch(parameters.convergence_method) {
      case fakecam::FakeCam_Frame:
        try{
          listener.lookupTransform(
              parameters.base_frame_id, parameters.convergence_frame_id,
              ros::Time(0), convergence_frame);
        } catch (tf::TransformException ex) {
          ROS_ERROR("Could not lookup convergence point: %s",ex.what());
        }

        break;
      case fakecam::FakeCam_Distance:
        // Use manual convergence distance
        convergence_angle = atan2(parameters.convergence_distance, parameters.separation/2.0);

        break;
      case fakecam::FakeCam_Angle:
        // Use manual convergence angle 
        convergence_angle = parameters.convergence_angle;
        break;
      default:
        ROS_ERROR("No convergence method set!");

    }
    left_frame.setRotation( tf::Quaternion(-convergence_angle, 0, 0) );
    right_frame.setRotation( tf::Quaternion(convergence_angle, 0, 0) );

    // Publish optical frames

    // Publish frames
    std::vector<tf::StampedTransform> transforms;

    left_frame.setOrigin( tf::Vector3(parameters.separation/2.0, 0.0, 0.0) );
    transforms.push_back(tf::StampedTransform(left_frame, ros::Time::now(), parameters.base_frame_id, "fake_cam_left_optical_link"));

    right_frame.setOrigin( tf::Vector3(-parameters.separation/2.0, 0.0, 0.0) );
    transforms.push_back(tf::StampedTransform(right_frame, ros::Time::now(), parameters.base_frame_id, "fake_cam_right_optical_link"));

    broadcaster.sendTransform(transforms);

    // Publish images
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "/fake_cam_left_optical_link";
    camera_info.header = msg.header;
    camera_info.P[0] = parameters.fov;
    camera_info.P[5] = parameters.fov;
    pub_left.publish(msg, camera_info);
    msg.header.frame_id = "/fake_cam_right_optical_link";
    camera_info.header = msg.header;
    pub_right.publish(msg, camera_info);

    ros::spinOnce();

    rate.sleep();
  }

  return 0;
}


