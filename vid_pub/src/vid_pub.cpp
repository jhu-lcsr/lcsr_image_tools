
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <camera_calibration_parsers/parse_ini.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <resource_retriever/retriever.h>
#include <terse_roscpp/param.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <camera_info_manager/camera_info_manager.h>


int main(int argc, char** argv) {

  namespace tparam = terse_roscpp::param;

  ros::init(argc, argv, "vid_pub");

  ros::NodeHandle nh, nh_private("~");
  image_transport::ImageTransport image_transport(nh);
  camera_info_manager::CameraInfoManager camera_info_manager(nh);
  image_transport::CameraPublisher camera_pub;

  std::string frame_id;
  std::string camera_name;
  std::string camera_info_url;
  std::string encoding;
  std::string video_uri;

  double fps = 25;
  bool auto_start = true;
  bool override_fps = false;
  bool loop = true;
  double start_time = 0.0,
         end_time = 0.0;

  override_fps = tparam::get(nh_private, 
      "fps",fps,
      "The framerate for the video stream in frames per second.");
  tparam::get(nh_private, 
      "auto_start",auto_start,
      "True if the video should start playing immediately.");
  tparam::get(nh_private, 
      "loop",loop,
      "True if the video should loop indefinitely.");
  tparam::get(nh_private, 
      "start_time",start_time,
      "Start time in seconds.");
  tparam::get(nh_private, 
      "end_time",end_time,
      "End time in seconds.");

  // Get TF Frame
  if(!nh_private.getParam("frame_id",frame_id)){
    frame_id = "/camera_frame";
    ROS_WARN_STREAM("No camera frame_id set, using frame \""<<frame_id<<"\".");
    nh_private.setParam("frame_id",frame_id);
  }

  // Get the camera parameters file
  nh_private.getParam("camera_info_url", camera_info_url);
  nh_private.getParam("camera_name", camera_name);
  nh_private.getParam("encoding", encoding);

  camera_info_manager.setCameraName(camera_name);

  if(camera_info_manager.validateURL(camera_info_url)) {
    camera_info_manager.loadCameraInfo(camera_info_url); 
  } else {
    ROS_WARN_STREAM("Camera info at: "<<camera_info_url<<" not found. Using an uncalibrated config.");
  }
  
  // Create ROS camera interface
  camera_pub = image_transport.advertiseCamera("camera/image_raw", 1);

  // Get the video 
  std::string video_file_path;
  tparam::require(nh_private,
      "video_file_path",video_file_path,
      "A ROS URI describing the location of a video file to be broadcasted.");

  cv::VideoCapture vid_cap(video_file_path);
  if(!override_fps) {
    fps = vid_cap.get(CV_CAP_PROP_FPS);
  }

  double n_frames = vid_cap.get(CV_CAP_PROP_FRAME_COUNT);

  ros::Rate rate(fps);
  ros::Time now;
  cv::Mat full_image;

  ROS_INFO_STREAM("Publishing camera streams from video file: "<<video_file_path<<" at "<<fps<<" FPS.");

  if(!vid_cap.set(CV_CAP_PROP_POS_MSEC,1000*start_time)) {
    ROS_ERROR("Could not seek to beginning of video!");
  }

  ros::Time playback_start_time = ros::Time::now();
  while(ros::ok()) {
    // Check if the video us opened
    if(!vid_cap.isOpened()) {
      ROS_ERROR("Video file is not opened!");
      break;
    }

    // Get a synchronized capture time
    now = ros::Time::now();

    // Grab a frame
    double frame = vid_cap.get(CV_CAP_PROP_POS_FRAMES);
    ROS_INFO_STREAM("Frame: "<<frame<<" of "<<n_frames);
    if(!vid_cap.read(full_image) || (now - playback_start_time).toSec() > (end_time-start_time)) {
      if(loop) {
        ROS_INFO("Reached end of file, re-winding to the beginning...");
        if(!vid_cap.set(CV_CAP_PROP_POS_AVI_RATIO,0.0)) {
          ROS_ERROR("Could not seek to beginning of video!");
        }
        vid_cap = cv::VideoCapture(video_file_path);
        playback_start_time = ros::Time::now();
        if(!vid_cap.set(CV_CAP_PROP_POS_MSEC,1000*start_time)) {
          ROS_ERROR("Could not seek to beginning of video!");
        }
        continue;
      } else {
        ROS_INFO("Reached end of file, stopping stream...");
        break;
      }
    }

    // Publish images
    // Create new image and camera_info messages
    sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image());
    sensor_msgs::CameraInfoPtr camera_info(new sensor_msgs::CameraInfo(camera_info_manager.getCameraInfo()));

    camera_info->header.frame_id = frame_id;
    camera_info->header.stamp = now;

    // Create a wrapper for the cv::Mat for publishing
    cv_bridge::CvImage image_bridge = cv_bridge::CvImage(
        camera_info->header,
        encoding, 
        full_image);

    // Convert the opencv image to a ros image message
    image_bridge.toImageMsg(*image_msg);

    // Publish the image and camera info
    camera_pub.publish(image_msg, camera_info);

    ros::spinOnce();

    rate.sleep();
  }

  return 0;
}


