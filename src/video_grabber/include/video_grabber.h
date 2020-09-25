//
// Copyright 2020 Human Dataware Lab. Co. Ltd.
// Created by Daiki Hayashi (hayashi.daiki@hdwlab.co.jp)
//

#ifndef ROS_VIDEO_GRABBER_H
#define ROS_VIDEO_GRABBER_H

#include <cmath>
#include <deque>
#include <ctime>
#include <cstdlib>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>

#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#ifndef NAN
#define NAN 0xffc00000
#endif

namespace video_grabber {

  class VideoGrabber {
  public:
    VideoGrabber(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    virtual ~VideoGrabber();
    void MainLoop();

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // publisher
    image_transport::Publisher publisher_;

    // variables
    bool isFirstContact_;
    int seekFrequency_;
    std::string pathToVideo_, pathToTimestamp_, frameId_;
    std::vector<uint64_t> timestamps_;
    uint64_t lastPublishedTimestamp_;
    cv::VideoCapture vc_;
    cv::Mat frame_;

    // internal functions
    void resetStates();
    void getParameters();
    bool loadTimestamps(std::string &pathToCsv);
    bool loadVideo(std::string &pathToVideo);
    bool seekFrameFromTimestamp(uint64_t &timestamp);
    void seekAndPublishImage(uint64_t &timestamp);
    bool isSeekable(uint64_t &timestamp);

  };

} // namespace video_grabber


#endif //ROS_VIDEO_GRABBER_H
