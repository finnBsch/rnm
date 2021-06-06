#include "ros/ros.h"
#include <iostream>
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>

using namespace std;

// create publishers globally
ros::Publisher rgb_pub;
ros::Publisher ir_pub;

//// callbacks:
// fetch current image from sensor_msgs and publish it
void fetchRgbImg(const sensor_msgs::ImageConstPtr& rgbmsg) {
  try {
    rgb_pub.publish(rgbmsg);
    cout << "rgb message published successfully" << endl;
  }
  catch (const std::exception&) {
    ROS_ERROR("Could not publish rgb message !!!!");
  }
}

void fetchIrImg(const sensor_msgs::ImageConstPtr& irmsg) {
  try {
    ir_pub.publish(irmsg);
    cout << "ir  message published successfully" << endl;
  }
  catch (const std::exception&) {
    ROS_ERROR("Could not publish ir message !!!!");
  }
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "get_images");
  ros::NodeHandle n; // node initialisation
  image_transport::ImageTransport it(n);

  // subscribe to camera topics
  // callbacks to fetch current images
  image_transport::Subscriber rgb_sub = it.subscribe("/k4a/rgb/image_raw", 1, fetchRgbImg);
  image_transport::Subscriber ir_sub = it.subscribe("/k4a/ir/image_raw", 1, fetchIrImg);

  // update publishers
  rgb_pub = n.advertise<sensor_msgs::Image>("calibration_rgb_img", 1);
  ir_pub = n.advertise<sensor_msgs::Image>("calibration_ir_img", 1);

  while (ros::ok()) { // didnt work
    cout << "Press key to select most recent frame" << endl;
    // wait until any key gets pressed
    while (cin.ignore()) {
      ros::spinOnce();
    }
  }
  return 0;
}