#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/JointState.h>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const sensor_msgs::JointState& joint_states)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::fromROSMsg(*cloud_msg, *cloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZRGB> ());

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();


  float theta = M_PI/4; // The angle of rotation in radians
  transform (0,0) = std::cos (theta);
  transform (0,1) = -sin(theta);
  transform (1,0) = sin (theta);
  transform (1,1) = std::cos (theta);
  //    (row, column)

  // Define a translation of 2.5 meters on the x axis.
  transform (0,3) = 2.5;

  // Print the transformation
  printf ("Method #1: using a Matrix4f\n");
  std::cout << transform << std::endl;

  pcl::transformPointCloud (*cloud, *cloud_transformed, transform);

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud_transformed, output);

  pub.publish (output);
}

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "PCMatrixTransform");
  ros::NodeHandle nh;

  message_filters::Subscriber<sensor_msgs::PointCloud2> filtered_cloud(nh, "/filtered_clouds", 1);
  message_filters::Subscriber<sensor_msgs::JointState> joint_states(nh, "/joint_states", 1);
  //ros::Subscriber sub = nh.subscribe("/filtered_clouds", 1, cloud_cb);

  TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::JointState> sync(filtered_cloud, joint_states, 10);

  sync.registerCallback(boost::bind(&cloud_cb, _1, _2));

  pub = nh.advertise<sensor_msgs::PointCloud2> ("transformed_clouds", 1);

  ros::spin ();
}

