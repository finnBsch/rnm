//
// Created by lars on 02.06.21.
//

#ifndef SRC_IMAGEHANDLER_H
#define SRC_IMAGEHANDLER_H
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
using namespace std;
class imagehandler {
 private:
  ros::NodeHandle n;
  image_transport::Subscriber sub_rgb;
  image_transport::Subscriber sub_ir;
  ros::Publisher pub_rgb;
  ros::Publisher pub_ir;
  image_transport::ImageTransport* it;

  void publish_rgb(const sensor_msgs::ImageConstPtr& msg);
  void publish_ir(const sensor_msgs::ImageConstPtr& msg);
 public:
  imagehandler(string rgbTopic, string irTopic, string rgbTopicPub,  string irTopicPub);
};

#endif  // SRC_IMAGEHANDLER_H
