//
// Created by lars on 02.06.21.
#include "imagehandler.h"
#include "calibration/imagepair.h"

imagehandler::imagehandler(string rgbTopic, string irTopic, string rgbTopicPub,  string irTopicPub){
  it = new image_transport::ImageTransport(n);
  pub_rgb = n.advertise<sensor_msgs::Image>(rgbTopicPub, 1);
  pub_ir = n.advertise<sensor_msgs::Image>(irTopicPub, 1);
  sub_rgb = it->subscribe(rgbTopic, 1, &imagehandler::publish_rgb, this);
  sub_ir = it->subscribe(irTopic, 1, &imagehandler::publish_ir, this);
}
void imagehandler::publish_rgb(const sensor_msgs::ImageConstPtr& msg){
  pub_rgb.publish(msg);
}
void imagehandler::publish_ir(const sensor_msgs::ImageConstPtr& msg){
  pub_ir.publish(msg);
}