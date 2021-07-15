#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include "point_cloud_registration/PCJScombined.h"

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "PointCloudRecording");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("JS", 1);

  for (int i = 0; i < 14; ++i) {
    std::cin.ignore();
    //sensor_msgs::PointCloud2ConstPtr PC = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("k4a/points2");
    sensor_msgs::JointStateConstPtr JS = ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");

    //point_cloud_registration::PCJScombined PCJS;
    //PCJS.PC = *PC;
    //PCJS.JS = *JS;

    pub.publish(JS);
    std::cout << "message published" << std::endl;
  }

}