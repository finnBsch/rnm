//
// Created by lars on 02.06.21.
//
#include "ros/ros.h"
#include <iostream>
#include "imagehandler.h"
#include "sensor_msgs/Image.h"
using namespace std;
int main(int argc, char **argv) {
  ros::init(argc, argv, "test");
  imagehandler myImageHandler("", "","","");
  cout << "Press key to select most recent frame" << endl;
  while(ros::ok()){
    if(true){//enter gedrückt
      sensor_msgs::ImageConstPtr img;
     img = *(ros::topic::waitForMessage<sensor_msgs::ImageConstPtr >("",ros::Duration(10)));
    }
  }
  while(cin.ignore()) {
    ros::spinOnce();
  }
  return 0;
}