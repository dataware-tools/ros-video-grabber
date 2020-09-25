/*
 Copyright 2020 Human Dataware Lab. Co. Ltd.
 Created by Daiki Hayashi (hayashi.daiki@hdwlab.co.jp)
 */


#include <ros/ros.h>
#include <video_grabber.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "video_grabber");
  ros::NodeHandle nh(""), pnh("~");
  video_grabber::VideoGrabber handler(nh, pnh);
  handler.MainLoop();

  return (0);
}
