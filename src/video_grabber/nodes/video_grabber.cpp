/*
 Copyright 2020 Human Dataware Lab. Co. Ltd.
 Created by Daiki Hayashi (hayashi.daiki@hdwlab.co.jp)
 */


#include <ros/ros.h>
#include <video_grabber.h>

namespace video_grabber {

  VideoGrabber::VideoGrabber(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    nh_ = nh;
    pnh_ = pnh;

    // initialize variables here
    isFirstContact_ = true;
    seekFrequency_ = 60;
    lastPublishedTimestamp_ = 0;
  }

  VideoGrabber::~VideoGrabber()
  {
    // deconstruct
    if (vc_.isOpened()) vc_.release();
  }

  void VideoGrabber::getParameters()
  {
    pnh_.param<std::string>("path_to_video", pathToVideo_, "");
    pnh_.param<std::string>("path_to_timestamp", pathToTimestamp_, "");
    pnh_.param<std::string>("frame_id", frameId_, "camera");
    pnh_.param<int>("seek_frequency", seekFrequency_, 60);
  }

  void VideoGrabber::resetStates()
  {
    ROS_DEBUG_STREAM(__func__);

    isFirstContact_ = true;
    if (vc_.isOpened()) vc_.set(1, 0);
    lastPublishedTimestamp_ = 0;
  }

  bool VideoGrabber::loadTimestamps(std::string &pathToCsv)
  {
    ROS_INFO_STREAM("Loading timestamp: " << pathToCsv);

    // init
    std::vector<uint64_t> timestamps;

    // load file
    std::ifstream ifs(pathToCsv.c_str());
    if(!ifs){
      ROS_ERROR_STREAM("Error: Could not open file " << pathToCsv);
      return false;
    }

    // line by line
    std::string row;
    while(getline(ifs, row)){
      std::string element;
      std::istringstream stream(row);

      // init
      std::vector<std::string> cols;

      while(getline(stream, element, ',')){
        cols.push_back(element);
      }

      // append
      timestamps.push_back(stoul(cols[0]));
    }

    // substitute
    timestamps_ = timestamps;

    // display
    ROS_INFO_STREAM("Loaded timestamp.");

    return true;
  }

  bool VideoGrabber::loadVideo(std::string &pathToVideo)
  {
    ROS_INFO_STREAM("Loading video file: " << pathToVideo);

    if (vc_.isOpened()) {
      vc_.release();
    }

    vc_.open(pathToVideo);

    if(!vc_.isOpened()) {
      ROS_ERROR_STREAM("ERROR while loading video file: cannot open the file");
      return false;
    }

    ROS_INFO_STREAM("Loaded video capture.");

    return true;
  }

  bool VideoGrabber::seekFrameFromTimestamp(uint64_t &timestamp)
  {
    // get current frame index
    auto currentFrame = uint64_t(vc_.get(1));

    // in case timestamp moved backward
    if (timestamp < timestamps_[currentFrame]) {
      ROS_WARN_STREAM("Timestamp moved backward");
      resetStates();
      return false;
    }

    // in case the corresponding frame is the current frame
    if ((timestamp >= timestamps_[currentFrame]) && (timestamp <= timestamps_[currentFrame+1])) {
      return false;
    }

    // search for small area
    for (uint64_t i=currentFrame+1; i<(currentFrame+300); i++) {
      if (i > 0 && i < timestamps_.size()) {
        vc_ >> frame_;
        if ((timestamp >= timestamps_[i]) && (timestamp <= timestamps_[i+1])) {
          return true;
        }
      }
    }

    ROS_WARN_STREAM("Searching for the corresponding frame");

    // search for the whole frames
    for (uint64_t i=0; i<timestamps_.size(); i++) {
      if ((timestamp >= timestamps_[i]) && (timestamp <= timestamps_[i+1])) {
        double findex = i * 1.0;
        vc_.set(1, findex);
        vc_ >> frame_;
        return true;
      }
    }

    ROS_ERROR_STREAM("No corresponding frame found");

    return false;
  }

  void VideoGrabber::seekAndPublishImage(uint64_t &timestamp)
  {
    if (vc_.isOpened()) {
      if (!seekFrameFromTimestamp(timestamp)) return;

      // get the true-timestamp
      auto t = timestamps_[uint64_t(vc_.get(1))];

      // publish image
      ROS_DEBUG_STREAM("Publishing an image");
      std_msgs::Header header;
      header.frame_id = frameId_;
      header.stamp = ros::Time(uint32_t(t / std::pow(10, 3)), uint32_t((t % 1000) * std::pow(10, 6)));
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", frame_).toImageMsg();
      publisher_.publish(msg);

      // TODO: Support image compression

      // update state
      lastPublishedTimestamp_ = timestamp;

    }else {
      ROS_ERROR_STREAM("Video file is not loaded");
    }
  }

  bool VideoGrabber::isSeekable(uint64_t &timestamp)
  {
    if (!vc_.isOpened()) return false;
    if (timestamps_.size() < 2) return false;
    if (timestamps_.front() > timestamp || timestamps_.back() < timestamp) {
      if (isFirstContact_) {
        ROS_WARN_STREAM("Current time (" << timestamp << ") is outside the video time-range.");
        ROS_WARN_STREAM("Video time-range: " << timestamps_.front() << " - " << timestamps_.back() << ".");
        isFirstContact_ = false;
      }
      return false;
    }
    return true;
  }

  void VideoGrabber::MainLoop()
  {
    // get parameters
    getParameters();

    // check parameters
    if (pathToTimestamp_.empty()) {
      ROS_ERROR_STREAM("Please set ros-param 'path_to_timestamp'");
      exit(1);
    }
    if (pathToVideo_.empty()) {
      ROS_ERROR_STREAM("Please set ros-param 'path_to_video'");
      exit(1);
    }

    // load csv
    if (!loadTimestamps(pathToTimestamp_)) {
      ROS_ERROR_STREAM("Failed to load timestamp");
      exit(1);
    }

    // load video
    if (!loadVideo(pathToVideo_)) {
      ROS_ERROR_STREAM("Failed to load video");
      exit(1);
    }

    // check number of frames
    if (vc_.get(cv::CAP_PROP_FRAME_COUNT) != timestamps_.size()) {
      ROS_ERROR_STREAM("Number of frames mismatched between timestamp and video");
      ROS_ERROR_STREAM("# of frames in video: " << vc_.get(cv::CAP_PROP_FRAME_COUNT));
      ROS_ERROR_STREAM("# of frames in timestamps: " << timestamps_.size());
      exit(1);
    }

    // get node name
    std::string nodeName;
    nodeName = ros::this_node::getName();
    if (nodeName.empty()) nodeName = "video_grabber";

    // prepare publisher
    image_transport::ImageTransport it(nh_);
    publisher_ = it.advertise(nodeName, 1);

    // set frequency
    ros::Rate loop_rate(seekFrequency_);

    // start seeking and publishing video
    while (ros::ok()) {
      ros::Time currentTime = ros::Time::now();
      if (!currentTime.isZero()) {
        auto timestamp = uint64_t(currentTime.toNSec() / std::pow(10, 6));

        // reset states if timestamp moved backward
        if (timestamp < lastPublishedTimestamp_) resetStates();

        // seek for the corresponding frame and publish the image
        if (isSeekable(timestamp)) {
          seekAndPublishImage(timestamp);
        }
      }
      ros::spinOnce();
      loop_rate.sleep();
    }

  }



} // end of namespace: video_grabber
