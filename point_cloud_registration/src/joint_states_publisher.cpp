#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include "point_cloud_registration/PCJScombined.h"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

ros::Publisher publisher;

void callback (const point_cloud_registration::PCJScombined::ConstPtr& pcjs)
{
  ROS_INFO("enterd callback");
  //pcl::PCLPointCloud2* received_cloud = new pcl::PCLPointCloud2;
  //pcl_conversions::toPCL(pcjs->PC, *received_cloud);
  //sensor_msgs::PointCloud2 output;
  //pcl_conversions::fromPCL(*received_cloud,output);
  publisher.publish(pcjs->JS);
}

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "joint_state_publisher");
  ros::NodeHandle nh("~");

  publisher = nh.advertise<sensor_msgs::JointState> ("joint_states", 50);

  ros::Subscriber sub = nh.subscribe("/PCJScombined", 1, &callback);

  ros::spin ();
}
