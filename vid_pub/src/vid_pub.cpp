
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

#include <dynamic_reconfigure/server.h>
#include <vid_pub/VidPubConfig.h>

class VidPub {

public: 
  VidPub(ros::NodeHandle nh, ros::NodeHandle nh_private_) :
    nh_(nh),
    nh_private_(nh_private_),
    image_transport_(nh),
    camera_info_manager_(nh),
    fps_(25),
    override_fps_(false),
    play_(true),
    loop_(true),
    start_time_(0.0),
    end_time_(0.0)
  {
    namespace tparam = terse_roscpp::param;

    override_fps_ = tparam::get(nh_private_, 
        "fps",fps_,
        "The framerate for the video stream in frames per second.");
    tparam::get(nh_private_, 
        "auto_start",play_,
        "True if the video should start playing immediately.");
    tparam::get(nh_private_, 
        "loop",loop_,
        "True if the video should loop indefinitely.");
    tparam::get(nh_private_, 
        "start_time",start_time_,
        "Start time in seconds.");
    tparam::get(nh_private_, 
        "end_time",end_time_,
        "End time in seconds.");

    // Get TF Frame
    if(!nh_private_.getParam("frame_id",frame_id_)){
      frame_id_ = "/camera_frame";
      ROS_WARN_STREAM("No camera frame_id set, using frame \""<<frame_id_<<"\".");
      nh_private_.setParam("frame_id",frame_id_);
    }

    // Get the camera parameters file
    nh_private_.getParam("camera_info_url", camera_info_url_);
    nh_private_.param<std::string>("camera_name", camera_name_, "default");
    nh_private_.param<std::string>("encoding", encoding_, "bgr8");

    camera_info_manager_.setCameraName(camera_name_);

    if(camera_info_manager_.validateURL(camera_info_url_)) {
      camera_info_manager_.loadCameraInfo(camera_info_url_); 
    } else {
      ROS_WARN_STREAM("Camera info at: "<<camera_info_url_<<" not found. Using an uncalibrated config.");
    }

    // Create ROS camera interface
    camera_pub_ = image_transport_.advertiseCamera("camera/image_raw", 1);

    // Get the video 
    tparam::require(nh_private_,
        "video_file_path",video_file_path_,
        "A ROS URI describing the location of a video file to be broadcasted.");

    // Set the dynamic reconfigure cb
    dynamic_reconfigure::Server<vid_pub::VidPubConfig>::CallbackType config_server_cb;
    config_server_cb = boost::bind(&VidPub::config_cb, this, _1, _2);
    config_server_.setCallback(config_server_cb);
  }

  void play_video() 
  {
    cv::VideoCapture vid_cap(video_file_path_);
    if(!override_fps_) {
      fps_ = vid_cap.get(CV_CAP_PROP_FPS);
    }

    double n_frames = vid_cap.get(CV_CAP_PROP_FRAME_COUNT);

    ros::Rate rate(fps_);
    ros::Time now;
    cv::Mat full_image;

    ROS_INFO_STREAM("Publishing camera streams from video file: "<<video_file_path_<<" at "<<fps_<<" FPS.");

    if(!vid_cap.set(CV_CAP_PROP_POS_MSEC,1000*start_time_)) {
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
      double msec = vid_cap.get(CV_CAP_PROP_POS_MSEC);
      ros::Time playback_time(msec/1000.0);

      // Debug status
      if(int(ceil(frame)) % int(ceil(fps_)) == 0 ) {
        ROS_DEBUG_STREAM("Start: "<<start_time_<<" End: "<<end_time_<<" Time: "<<msec/1000.0<<" Frame: "<<frame<<" of "<<n_frames);
      }

      // Only capture if play_ is enabled
      if(play_) {
        bool image_captured = vid_cap.read(full_image);
        bool before_play_range = (playback_time.toSec() - start_time_) < -(1.0/fps_);
        bool after_play_range = end_time_ > 1E-5 && (playback_time.toSec() - end_time_) > (1.0/fps_);
        
        if(!image_captured || before_play_range || after_play_range) {
          if(loop_ && !image_captured) {
            // Re-open the file
            vid_cap = cv::VideoCapture(video_file_path_);
            // Go around again to capture
            continue;
          } else if(before_play_range || (loop_ && after_play_range)) {
            // Seek to the beginning of the play range
            ROS_INFO("Seeking to the beginning of play range...");
            if(!vid_cap.set(CV_CAP_PROP_POS_MSEC,1000*start_time_)) {
              ROS_ERROR("Could not seek to beginning of play range!");
            }
            // Go around again to capture
            continue;
          } else {
            ROS_INFO("Reached end of play range, stopping stream...");
            break;
          }
        }
      }

      // Publish images
      // Create new image and camera_info messages
      sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image());
      sensor_msgs::CameraInfoPtr camera_info(new sensor_msgs::CameraInfo(camera_info_manager_.getCameraInfo()));

      camera_info->header.frame_id = frame_id_;
      camera_info->header.stamp = now;
      image_msg->header = camera_info->header;
      image_msg->encoding = encoding_;

      // Create a wrapper for the cv::Mat for publishing
      cv_bridge::CvImage image_bridge = cv_bridge::CvImage(
          camera_info->header,
          encoding_, 
          full_image);

      // Convert the opencv image to a ros image message
      image_bridge.toImageMsg(*image_msg);

      // Publish the image and camera info
      camera_pub_.publish(image_msg, camera_info);

      ros::spinOnce();

      rate.sleep();
    }
  }

private:
  ros::NodeHandle nh_, nh_private_;
  image_transport::ImageTransport image_transport_;
  camera_info_manager::CameraInfoManager camera_info_manager_;
  image_transport::CameraPublisher camera_pub_;

  dynamic_reconfigure::Server<vid_pub::VidPubConfig> config_server_;

  std::string frame_id_;
  std::string camera_name_;
  std::string camera_info_url_;
  std::string encoding_;
  std::string video_file_path_;

  double fps_;
  bool override_fps_;
  bool play_;
  bool loop_;
  double start_time_;
  double end_time_;

  void config_cb(vid_pub::VidPubConfig &config, uint32_t level)
  {
    play_ = config.play;
    loop_ = config.loop;
    start_time_ = std::max(0.0,config.start_time);
    end_time_ = (start_time_ < config.end_time) ? (config.end_time) : (0.0);
  }

};


int main(int argc, char** argv) {

  ros::init(argc, argv, "vid_pub");

  ros::NodeHandle nh, nh_private("~");

  VidPub vid_pub(nh,nh_private);

  vid_pub.play_video();

  return 0;
}


