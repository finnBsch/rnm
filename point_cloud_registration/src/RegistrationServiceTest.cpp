#include <ros/ros.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <thread>
#include <pcl/common/angles.h> // for pcl::deg2rad
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
//#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
//#include <pcl/registration/ndt.h>
#include <std_msgs/Float64MultiArray.h>
#include "point_cloud_registration/alignment_service.h"
#include <eigen_conversions/eigen_msg.h>

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "RegistrationServiceTest");
  ros::NodeHandle nh;
  //Let the publisher publish on topic transformed_clouds

  ros::Publisher publisher;
  publisher = nh.advertise<std_msgs::Float64MultiArray> ("STLtransformation", 1);
  point_cloud_registration::alignment_service srv;

  ros::ServiceClient client = nh.serviceClient<point_cloud_registration::alignment_service>("STLRegistration/alignment_service");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr stitched_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::io::loadPCDFile("/home/niklas/Documents/RNM/stitched_cloud.pcd", *stitched_cloud);
  pcl::toROSMsg(*stitched_cloud,srv.request.stitched_cloud);

  client.call(srv);

  Eigen::Matrix4f transform;
  transform << srv.response.alignment_transformation.data[0],srv.response.alignment_transformation.data[1],srv.response.alignment_transformation.data[2],srv.response.alignment_transformation.data[3],
      srv.response.alignment_transformation.data[4],srv.response.alignment_transformation.data[5],srv.response.alignment_transformation.data[6],srv.response.alignment_transformation.data[7],
      srv.response.alignment_transformation.data[8],srv.response.alignment_transformation.data[9],srv.response.alignment_transformation.data[10],srv.response.alignment_transformation.data[11],
      srv.response.alignment_transformation.data[12],srv.response.alignment_transformation.data[13],srv.response.alignment_transformation.data[14],srv.response.alignment_transformation.data[15];
  std::cout << transform(0) << transform(1) << transform(2) << transform(3);
}