#include "ros/ros.h"
#include <iostream>
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;


ros::NodeHandle n; // node initialisation
ros::Publisher rgb_pub = n.advertise<sensor_msgs::Image>("calibration_rgb_img", 1); //Note to self: maybe try it (im trans).
ros::Publisher ir_pub = n.advertise<sensor_msgs::Image>("calibration_ir_img", 1);
// global image matrices
// going to be published
//cv::Mat rgbImg;
//cv::Mat irImg;

//// callbacks:
// fetch current image from sensor_msgs and convert to cv::Mat
void fetchRgbImg(const sensor_msgs::ImageConstPtr& msg) {
  rgb_pub.publish(msg);
  // try {
  //     // convert image from ros msg to bgr8 (cv)
  //     rgbImg = cv_bridge::toCvShare(msg, "bgr8")->image;
  //
  //     std::cout << "rgb image converted successfully" << endl;
  //   }
  //   catch (cv_bridge::Exception &e) {
  //     std::cout << "Error: rgb image could not be converted !!!!" << endl;
  //   }
}

void fetchIrImg(const sensor_msgs::ImageConstPtr& msg) {
  ir_pub.publish(msg);
  // try {
  //     // convert image from ros msg to bgr8 (cv)
  //     rgbImg = cv_bridge::toCvShare(msg, "bgr8")->image;
  //
  //     std::cout << "ir image converted successfully" << endl;
  //   }
  //   catch (cv_bridge::Exception &e) {
  //     std::cout << "Error: ir image could not be converted !!!!" << endl;
  //   }
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "get_images");
  //ros::NodeHandle n; // node initialisation
  image_transport::ImageTransport it(n);

  // subscribe to camera topics
  // callbacks to fetch and convert current images
  image_transport::Subscriber rgb_sub = it.subscribe("/k4a/rgb/image_raw", 1, fetchRgbImg);
  image_transport::Subscriber ir_sub = it.subscribe("/k4a/ir/image_raw", 1, fetchIrImg);

  // publish images
  //ros::Publisher rgb_pub = n.advertise<sensor_msgs::Image>("calibration_rgb_img", 1); //Note to self: maybe try it (im trans).
  //ros::Publisher ir_pub = n.advertise<sensor_msgs::Image>("calibration_ir_img", 1);

  while (ros::ok()) {
    cout << "Press key to select most recent frame" << endl;
    // wait until any key gets pressed
    while (cin.ignore()) {
      ros::spinOnce();
    }
  }
  return 0;
}