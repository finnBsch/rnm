#include <iostream>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

//Public publisher
ros::Publisher publisher;

//for testing only: counter to see how many times the callback function has been called
int msg_counter = 0;


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const sensor_msgs::JointStateConstPtr& joint_states)
{
  //FILTERING
  //Initialize container for PC message, for the filtered cloud and pointer to PC message
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 pc2_cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (pc2_cloud_filtered);

  //TRANSFORMATION
  //Initialize container for input and output cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZRGB> ());

  //write filtered cloud in input cloud
  pcl::fromPCLPointCloud2(pc2_cloud_filtered, *cloud_filtered);

  //print out how many messages have been received in total
  msg_counter++;
  printf("Received messages: \n");
  std::cout << msg_counter;
  printf("\n");

  //definition of an example transformation matrix
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  float theta = M_PI/4;
  transform (0,0) = std::cos (theta);
  transform (0,1) = -sin(theta);
  transform (1,0) = sin (theta);
  transform (1,1) = std::cos (theta);
  transform (0,3) = 2.5;

  //print out the received joint states, for test purposes
  for (int i = 0; i < 7; ++i) {
    std::cout << joint_states->position[i];
  }

  //Transformation of the filtered cloud
  pcl::transformPointCloud (*cloud_filtered, *cloud_transformed, transform);

  //Define a PC for output and publish it
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud_transformed, output);
  publisher.publish (output);
}

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "PCMatrixTransform");
  ros::NodeHandle nh;

  //Let the publisher publish on topic transformed_clouds
  publisher = nh.advertise<sensor_msgs::PointCloud2> ("transformed_clouds", 1);

  //Subscribe to the PC and JointState topic, large queue size where all messages from the bag can fit in
  //-> adjust queue size later
  message_filters::Subscriber<sensor_msgs::PointCloud2> filtered_cloud(nh, "/points2", 150);
  message_filters::Subscriber<sensor_msgs::JointState> joint_states(nh, "/joint_states", 1100);

  //Initialize approximate time filter, to fit together PCs and joint states by time stamp
  //again a large queue size to make sure no messages get lost
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::JointState> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(150), filtered_cloud, joint_states);
  sync.registerCallback(boost::bind(&cloud_cb, _1, _2));

  ros::spin ();
}

