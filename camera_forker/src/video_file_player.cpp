

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
