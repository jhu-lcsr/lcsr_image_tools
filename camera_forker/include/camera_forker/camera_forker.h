#ifndef __CAMERA_FORKER_CAMERA_FORKER_H
#define __CAMERA_FORKER_CAMERA_FORKER_H

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <camera_calibration_parsers/parse_ini.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <resource_retriever/retriever.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

namespace camera_forker {
  class ForkedPublisher
  {
  public:
    ForkedPublisher(ros::NodeHandle nh, ros::NodeHandle nh_private, const std::string &camera_name);
    void publish(
        const sensor_msgs::ImageConstPtr &full_image_msg,
        const sensor_msgs::CameraInfoConstPtr &full_camera_info_msg);

  private:
    // Local nodehandle
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // Parameters
    std::string camera_name_;
    
    // Camera info
    bool use_predefined_camera_info_;
    std::string camera_info_url_;
    bool inherit_camera_info_;


    // ROI selection
    bool use_roi_;
    cv::Rect roi_;

    // AFfine Transform
    bool use_affine_;
    cv::Mat affine_transform_;
    cv::Size affine_transform_size_;

    std::string frame_id_;
    std::string encoding_;
    int pub_buffer_size_;

    // ROS Interface
    camera_info_manager::CameraInfoManager camera_info_manager_;
    image_transport::ImageTransport transport_;
    image_transport::CameraPublisher camera_pub_;

  };

  class CameraForker
  {
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
  public:
    typedef boost::ptr_vector<ForkedPublisher> ForkedPublisherList;

    CameraForker(ros::NodeHandle nh, ros::NodeHandle nh_private);

    void publish(
        const sensor_msgs::ImageConstPtr &image_msg,
        const sensor_msgs::CameraInfoConstPtr &camera_info_msg);

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    image_transport::ImageTransport image_transport_;
    image_transport::CameraSubscriber camera_sub_;
    std::vector<std::string> camera_fork_names_;
    ForkedPublisherList camera_forks_;

  };

}


#endif // ifndef __CAMERA_FORKER_CAMERA_FORKER_H
