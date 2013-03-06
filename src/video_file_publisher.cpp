
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
#include <terse_roscpp/param.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

class FakeCamera {
private:
  // Local nodehandle
  ros::NodeHandle nh_;

  // Parameters
  std::string camera_name_;
  std::string calibration_file_uri_;
  bool use_roi_;
  cv::Rect roi_;
  std::string frame_id_;
  int pub_buffer_size_;

  // Loaded data / working data
  sensor_msgs::CameraInfo camera_info_msg_;
  sensor_msgs::Image img_msg_;

  image_transport::ImageTransport transport_;
  image_transport::CameraPublisher pub_;

public:
  FakeCamera(ros::NodeHandle nh, const std::string &camera_name)
    : nh_(nh, camera_name),
    camera_name_(camera_name),
    calibration_file_uri_(""),
    use_roi_(false),
    roi_(0,0,0,0),
    frame_id_(""),
    pub_buffer_size_(5),
    transport_(nh_),
    pub_()
  {
    namespace tparam = terse_roscpp::param;


    tparam::require(nh_, "calibration_file_uri", calibration_file_uri_,
        "A ROS URI describing the location of the camera calibration file for the camera info.");

    // Image properties
    std::vector<double> roi(4,-1);
    use_roi_ = tparam::get(nh_, "roi", roi,
        "The ROI (region of interest) of the video stream. Array: [x,y,width,height]");
    if(use_roi_) {
      if(roi.size() == 4) {
        roi_ = cv::Rect(roi[0],roi[1],roi[2],roi[3]);
      } else {
        ROS_ERROR_STREAM("ROI parameter is not 4 elements long! Using full image instead. Parameter is: "<<nh_.getNamespace()<<"/roi");
        use_roi_ = false;
      }
    }
    tparam::get(nh_, "encoding", img_msg_.encoding,
        "The color encoding for this camera.");
    tparam::require(nh_, "frame_id", img_msg_.header.frame_id,
        "The TF frame for this camera.");

    // Publisher properties
    tparam::get(nh_, "pub_buffer_size", pub_buffer_size_,
        "True if the image data is big-endian.");

    // Get calibration info
    this->get_camera_info(camera_info_msg_);

    // create image transport and publishers
    pub_ = transport_.advertiseCamera(camera_name_, pub_buffer_size_);
  }

  void publish(const cv::Mat &full_image, const ros::Time &now)
  {
    // Update header stamp
    img_msg_.header.stamp = now;

    // The image to broadcast
    cv::Mat mat;

    // Get the sub-image if an ROI is defined
    if(use_roi_) {
      mat = cv::Mat(full_image, roi_);
    } else {
      mat = full_image;
    }

    // Create a wrapper for the cv::Mat for publishing
    cv_bridge::CvImage cv_image = cv_bridge::CvImage(
        img_msg_.header,
        img_msg_.encoding, 
        mat);
    
    // Convert the opencv image to a ros image message
    cv_image.toImageMsg(img_msg_);

    // Publish the image and camera info
    pub_.publish(img_msg_, camera_info_msg_);
  }

private:

  void get_camera_info(sensor_msgs::CameraInfo &camera_info)
  {
    // Retrieve the camera calibration file
    resource_retriever::Retriever retriever;
    resource_retriever::MemoryResource resource;

    try {
      resource = retriever.get(calibration_file_uri_); 
    } catch (resource_retriever::Exception& e) {
      ROS_FATAL_STREAM("Failed to retrieve camere calibration file from \""<<calibration_file_uri_<<"\" error: "<<e.what());
      throw e;
    }

    // Get the camera info
    std::string camera_name;// TODO: Should we throw this away?
    if (camera_calibration_parsers::parseCalibrationIni(
          std::string((const char*)resource.data.get(),resource.size),
          camera_name,
          camera_info))
    {
      ROS_INFO_STREAM("Successfully parsed camera calibration from \""<<calibration_file_uri_<<"\"");
    } else {
      std::ostringstream oss;
      oss<<"Could not parse camera calibration from \""<<calibration_file_uri_<<"\"";
      throw ros::Exception(oss.str());
    }
  }

};


int main(int argc, char** argv) {

  namespace tparam = terse_roscpp::param;

  ros::init(argc, argv, "video_file_publisher");
  ros::NodeHandle nh_private("~");

  std::string video_uri;
  double fps = 25;
  bool auto_start = false;
  bool override_fps = false;

  tparam::require(nh_private,
      "video_file_uri",video_uri,
      "A ROS URI describing the location of a video file to be broadcasted.");
  override_fps = tparam::get(nh_private, 
      "fps",fps,
      "The framerate for the video stream in frames per second.");
  tparam::get(nh_private, 
      "auto_start",auto_start,
      "True if the video should start playing immediately.");

  /**
   * Get the camera list.
   *
   *  cameras:
   *    left:
   *      frame_id: stereo_left
   *      roi: [0,0,320,240]
   *      calibration_file: file://path/to/left/cfg
   *    right:
   *      frame_id: stereo_right
   *      roi: [320,0,320,240]
   *      calibration_file: file://path/to/right.cfg
   */

  std::vector<std::string> camera_names;
  tparam::children(nh_private, "cameras", camera_names,
      "The list of cameras to be spawned by this node.", 
      true);

  // Construct the cameras
  typedef boost::ptr_vector<FakeCamera> FakeCameraList;
  FakeCameraList cameras(0);

  for(std::vector<std::string>::const_iterator camera_name = camera_names.begin();
      camera_name != camera_names.end();
      ++camera_name)
  {
    cameras.push_back(
          new FakeCamera(ros::NodeHandle(nh_private,"cameras"), *camera_name));
  }

  // Retrieve the video file
  resource_retriever::Retriever retriever;
  resource_retriever::MemoryResource resource;

  try {
    resource = retriever.get(video_uri); 
  } catch (resource_retriever::Exception& e) {
    ROS_FATAL_STREAM("Failed to retrieve video file from \""<<video_uri<<"\" error: "<<e.what());
    throw e;
  }

  // Construct opencv video capture and open the file
  cv::VideoCapture vid_cap(std::string((const char*)resource.data.get(),resource.size));
  if(!override_fps) {
    fps = vid_cap.get(CV_CAP_PROP_FPS);
  }

  ros::Rate rate(fps);
  ros::Time now;
  cv::Mat full_image;

  ROS_INFO_STREAM("Publishing fake camera streams from video file...");

  while(ros::ok()) {
    if(vid_cap.isOpened()) {
      ROS_ERROR("Video file is not opened!");
      break;
    }

    // Get a synchronized capture time
    now = ros::Time::now();

    // Grab a frame
    vid_cap >> full_image;

    // Publish images
    for(FakeCameraList::iterator camera = cameras.begin();
        camera != cameras.end();
        ++camera)
    {
      camera->publish(full_image, now);
    }

    ros::spinOnce();

    rate.sleep();
  }

  return 0;
}


