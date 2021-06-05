#include "ros/ros.h"
#include "imageprocessor.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "test");
  imageprocessor myImageProcessor("","") ; //
  ros::spin();
  return EXIT_SUCCESS;
}