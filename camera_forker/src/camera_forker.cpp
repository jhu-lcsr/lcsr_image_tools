
#include <ros/ros.h>

#include <terse_roscpp/param.h>

#include <camera_forker/camera_forker.h>

namespace camera_forker {
  CameraForker::CameraForker(ros::NodeHandle nh, ros::NodeHandle nh_private) :
    nh_(nh),
    nh_private_(nh_private),
    image_transport_(nh),
    camera_fork_names_(0),
    camera_forks_(0)
  {
    namespace tparam = terse_roscpp::param;

    tparam::children(nh_private_, "cameras", camera_fork_names_,
        "The list of cameras to be spawned by this node.", 
        true);

    // Construct the fork publishers
    for(std::vector<std::string>::const_iterator camera_fork_name = camera_fork_names_.begin();
        camera_fork_name != camera_fork_names_.end();
        ++camera_fork_name)
    {
      camera_forks_.push_back(
          new ForkedPublisher(
            ros::NodeHandle(nh_,"cameras"), 
            ros::NodeHandle(nh_private_,"cameras"), 
            *camera_fork_name));
    }

    // Construct the subscriber
    camera_sub_ = image_transport_.subscribe("camera/image_raw",1,&CameraForker::publish,this);
  }

  void CameraForker::publish(const sensor_msgs::ImageConstPtr &image_msg)
  {
    for(ForkedPublisherList::iterator camera_fork = camera_forks_.begin();
        camera_fork != camera_forks_.end();
        ++camera_fork)
    {
      camera_fork->publish(image_msg);
    }
  }

  ForkedPublisher::ForkedPublisher(
      ros::NodeHandle nh,
      ros::NodeHandle nh_private,
      const std::string &camera_name) : 
    nh_(nh, camera_name),
    nh_private_(nh_private, camera_name),
    camera_name_(camera_name),
    camera_info_url_(""),
    use_roi_(false),
    roi_(0,0,0,0),
    use_affine_(false),
    affine_transform_(cv::Mat::eye(2,3,CV_32FC1)),
    frame_id_(""),
    pub_buffer_size_(5),
    camera_info_manager_(nh),
    transport_(nh_),
    camera_pub_()
  {
    namespace tparam = terse_roscpp::param;

    tparam::get(nh_private_, "camera_info_url_", camera_info_url_,
        "A ROS URI describing the location of the camera calibration file for the camera info.");

    camera_info_manager_.setCameraName(camera_name_);

    if(camera_info_manager_.validateURL(camera_info_url_)) {
      camera_info_manager_.loadCameraInfo(camera_info_url_); 
    } else {
      ROS_WARN_STREAM("Camera info at: "<<camera_info_url_<<" not found. Using an uncalibrated config.");
    }

    // Image properties
    std::vector<int> roi(4,-1);
    use_roi_ = tparam::get(nh_private_, "roi", roi,
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
    use_affine_ = nh_private_.hasParam("affine");
    if(use_affine_) {
      ros::NodeHandle nh_affine = ros::NodeHandle(nh_private_,"affine");

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

    tparam::get(nh_private_, "encoding", encoding_,
        "The color encoding for this camera.");
    tparam::require(nh_private_, "frame_id", frame_id_,
        "The TF frame for this camera.");

    // Publisher properties
    tparam::get(nh_private_, "pub_buffer_size", pub_buffer_size_,
        "The size of the publisher buffer.");

    // create image transport and publishers
    camera_pub_ = transport_.advertiseCamera("image_raw", pub_buffer_size_);
  }

  void ForkedPublisher::publish(const sensor_msgs::ImageConstPtr& full_image_msg)
  {
    // Create new image and camera_info messages
    sensor_msgs::ImagePtr forked_image_msg(new sensor_msgs::Image());
    sensor_msgs::CameraInfoPtr forked_camera_info(new sensor_msgs::CameraInfo(camera_info_manager_.getCameraInfo()));

    // Copy header from forked camera
    forked_camera_info->header.stamp = full_image_msg->header.stamp;
    forked_camera_info->header.frame_id = frame_id_;

    // The image to broadcast
    cv_bridge::CvImageConstPtr full_image_bridge = cv_bridge::toCvShare(full_image_msg, encoding_);
    cv::Mat forked_image;

    // Get the sub-image if an ROI is defined
    if(use_roi_) {
      forked_image = cv::Mat(full_image_bridge->image, roi_);
    } else if(use_affine_) {
      cv::warpAffine(
          full_image_bridge->image,
          forked_image, 
          affine_transform_, 
          affine_transform_size_,
          cv::INTER_LINEAR,
          cv::BORDER_CONSTANT);
    } else {
      forked_image = full_image_bridge->image;
    }

    // Create a wrapper for the cv::Mat for publishing
    cv_bridge::CvImage forked_image_bridge = cv_bridge::CvImage(
        forked_camera_info->header,
        encoding_, 
        forked_image);

    // Convert the opencv image to a ros image message
    forked_image_bridge.toImageMsg(*forked_image_msg);

    // Publish the image and camera info
    camera_pub_.publish(forked_image_msg, forked_camera_info);
  }
}
