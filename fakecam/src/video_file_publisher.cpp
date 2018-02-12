
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
  bool use_affine_;
  cv::Mat affine_transform_;
  cv::Size affine_transform_size_;
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
    use_affine_(false),
    affine_transform_(cv::Mat::eye(2,3,CV_32FC1)),
    frame_id_(""),
    pub_buffer_size_(5),
    transport_(nh_),
    pub_()
  {
    namespace tparam = terse_roscpp::param;


    tparam::require(nh_, "calibration_file_uri", calibration_file_uri_,
        "A ROS URI describing the location of the camera calibration file for the camera info.");

    // Image properties
    std::vector<int> roi(4,-1);
    use_roi_ = tparam::get(nh_, "roi", roi,
        "The ROI (region of interest) of the video stream. Array: [x,y,width,height]");
    if(use_roi_) {
      if(roi.size() == 4) {
        roi_ = cv::Rect(roi[0],roi[1],roi[2],roi[3]);
      } else {
        ROS_WARN_STREAM("ROI parameter is not 4 elements long! Using full image instead. Parameter is: "<<nh_.getNamespace()<<"/roi");
        use_roi_ = false;
      }
    }

    // Affine transformation
    use_affine_ = nh_.hasParam("affine");
    if(use_affine_) {
      ros::NodeHandle nh_affine = ros::NodeHandle(nh_,"affine");

      std::vector<double> translation(2,0.0), scale(2,1.0);
      double angle = 0.0;
      std::vector<int> size(2,0);

      tparam::get(nh_affine, "translation", translation, "The affine translation. Array: [x,y]");
      tparam::get(nh_affine, "rotation_angle", angle, "The affine rotation.");
      tparam::get(nh_affine, "scale", scale, "The affine scale. Array: [sx,sy]");
      tparam::get(nh_affine, "size", size, "The size of the transformed image. Array: [width,height]");

      // Construct translation
      cv::Mat T = cv::Mat::eye(3,3,CV_32F);
      T.at<float>(0,2) = translation[0];
      T.at<float>(1,2) = translation[1];

      // Construct rotation
      cv::Mat R = cv::Mat::eye(3,3,CV_32F);
      R.at<float>(0,0) = cos(angle); R.at<float>(0,1) = sin(angle);
      R.at<float>(1,0) = -sin(angle); R.at<float>(1,1) = cos(angle);

      cv::Mat S = cv::Mat::eye(3,3,CV_32F);
      S.at<float>(0,0) = scale[0];
      S.at<float>(1,1) = scale[1];

      // Store the non-homogenous affine transform
      affine_transform_ = (S*R*T)(cv::Rect(0,0,3,2));
      affine_transform_size_ = cv::Size(size[0],size[1]);

      ROS_INFO_STREAM("Loaded affine transform: S*R*T\n"<<S<<"\n"<<R<<"\n"<<T<<"\n = "<<affine_transform_);
    }

    tparam::get(nh_, "encoding", img_msg_.encoding,
        "The color encoding for this camera.");
    tparam::require(nh_, "frame_id", img_msg_.header.frame_id,
        "The TF frame for this camera.");

    // Publisher properties
    tparam::get(nh_, "pub_buffer_size", pub_buffer_size_,
        "The size of the publisher buffer.");

    // Get calibration info
    this->get_camera_info(camera_info_msg_);

    // create image transport and publishers
    pub_ = transport_.advertiseCamera("image", pub_buffer_size_);
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
    } else if(use_affine_) {
      cv::warpAffine(full_image, mat, 
          affine_transform_, 
          affine_transform_size_,
          cv::INTER_LINEAR,
          cv::BORDER_CONSTANT);
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


