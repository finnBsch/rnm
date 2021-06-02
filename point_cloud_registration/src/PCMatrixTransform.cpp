#include <iostream>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
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

using namespace Eigen;

//global cloud to store the stitched cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr stitched_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());

//values to calculate forward kinematics
static VectorXf a(8);
static VectorXf d(8);
static VectorXf alpha(8);

Matrix4f get_transformationmatrix(const float theta, const float a, const float d, const float alpha){
  Matrix4f ret_mat(4,4);
  ret_mat << cos(theta), -sin(theta), 0, a,
      sin(theta) * cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d * sin(alpha),
      sin(theta) * sin(alpha), cos(theta)*sin(alpha), cos(alpha), d * cos(alpha),
      0, 0, 0, 1;
  return ret_mat;
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const sensor_msgs::JointStateConstPtr& joint_states)
{
  //print out how many messages have been received in total
  msg_counter++;
  printf("\nReceived messages: ");
  std::cout << msg_counter;
  printf("\n");

  std::cout << PCL_VERSION << std::endl;

  //only take every 20th message for stitching
  if (msg_counter%20 == 0){


  //SUBSAMPLE---------------------------------------------------------------------------------------------------------------
  //Initialize container for PC message, for the filtered cloud and pointer to PC message
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 pc2_cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter (pc2_cloud_filtered);


  //TRANSFORMATION---------------------------------------------------------------------------------------------------------
  //Initialize container for input and output cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZRGB> ());

  //write filtered cloud in input cloud
  pcl::fromPCLPointCloud2(pc2_cloud_filtered, *cloud_filtered);

  //hand-eye calibration matrix
  Matrix4f handeye;
  handeye << -0.0342177, -0.0224303,   0.999141,  0.0357424,
          -0.998979, -0.0285611,  -0.034854, -0.0232153,
          0.0293307,  -0.999324, -0.0214294,  0.0547717,
          0,          0,          0,          1;


  //calculate the transformation matrix from robot base to camera
  std::array<Matrix4f, 8> a_;
  for(int i  = 0; i<8; i++){
    a_.at(i) = get_transformationmatrix(joint_states->position[i], a(i), d(i), alpha(i));
  }
  Matrix4f transform = a_.at(0)*a_.at(1)*a_.at(2)*a_.at(3)*a_.at(4)*a_.at(5)*a_.at(6)*a_.at(7)*handeye;

  //Transformation of the filtered cloud
  pcl::transformPointCloud (*cloud_filtered, *cloud_transformed, transform);


  //CROPPING-------------------------------------------------------------------------------------------------------------------------
  //Crop the point cloud, so it mainly contains the skeleton
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cropped (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::CropBox<pcl::PointXYZRGB> crop;
  crop.setMin(Vector4f(0.15, -0.5, 0.02, 1.0));
  crop.setMax(Vector4f(0.7, 0.5, 0.3, 1.0));
  crop.setInputCloud(cloud_transformed);
  crop.filter(*cloud_cropped);


  //ICP--------------------------------------------------------------------------------------------------------------------------------
  //using ICP for fine alignment

  //If condition to test if the total stitched cloud is still empty. If so fill it with the arrived cloud.
  // Else stitch the arrived cloud to the total stitched cloud
  if (stitched_cloud->empty() == 1){
    std::cout << "first initialization";
    *stitched_cloud = *cloud_cropped;
  }else{
    //Set clouds for the ICP algorithm
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr goal_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputSource(cloud_cropped);
    icp.setInputTarget(stitched_cloud);

    //ICP parameters (examples), we still need to adjust them
    icp.setEuclideanFitnessEpsilon(0.000000000001);
    icp.setTransformationEpsilon(0.0000001);
    icp.setMaximumIterations(100);

    //align new cloud to stiched cloud and add to the total stiched cloud.
    icp.align(*goal_cloud);
    *stitched_cloud += *goal_cloud;
  }

  //Define a PC for output and publish it
  sensor_msgs::PointCloud2 output;

  pcl::toROSMsg(*stitched_cloud, output);

  publisher.publish (output);
  }
}

int
main (int argc, char** argv)
{
  //Denavit Hartenberg values
  a << 0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0;
  d << 0.333, 0, 0.316, 0, 0.384, 0, 0, 0.107;
  alpha << 0, -M_PI/2, M_PI/2, M_PI/2, -M_PI/2, M_PI/2, M_PI/2, 0;

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

