#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include "point_cloud_registration/PCJScombined.h"

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "PointCloudRecording");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<point_cloud_registration::PCJScombined>("PCJScombined", 1);

  for (int i = 0; i < 20; ++i) {
    std::cin.ignore();
    sensor_msgs::PointCloud2ConstPtr PC = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("k4a/points2");
    sensor_msgs::JointStateConstPtr JS = ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");

    point_cloud_registration::PCJScombined PCJS;
    PCJS.PC = *PC;
    PCJS.JS = *JS;

    pub.publish(PCJS);
    std::cout << "message publihsed" << std::endl;
  }

}