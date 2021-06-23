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
#include <visualization_msgs/Marker.h>

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "RegistrationServiceTest");
  ros::NodeHandle nh;
  //Let the publisher publish on topic transformed_clouds

  ros::Publisher publisher;
  publisher = nh.advertise<std_msgs::Float64MultiArray> ("STLtransformation", 1);
  point_cloud_registration::alignment_service srv;

  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::ServiceClient client = nh.serviceClient<point_cloud_registration::alignment_service>("STLRegistration2/alignment_service");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr stitched_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::io::loadPCDFile("/home/niklas/Documents/RNM/stitched_cloud_backup.pcd", *stitched_cloud);
  pcl::toROSMsg(*stitched_cloud,srv.request.stitched_cloud);

  client.call(srv);

  Eigen::Matrix4f transform;
  transform << srv.response.alignment_transformation.data[0],srv.response.alignment_transformation.data[1],srv.response.alignment_transformation.data[2],srv.response.alignment_transformation.data[3],
      srv.response.alignment_transformation.data[4],srv.response.alignment_transformation.data[5],srv.response.alignment_transformation.data[6],srv.response.alignment_transformation.data[7],
      srv.response.alignment_transformation.data[8],srv.response.alignment_transformation.data[9],srv.response.alignment_transformation.data[10],srv.response.alignment_transformation.data[11],
      srv.response.alignment_transformation.data[12],srv.response.alignment_transformation.data[13],srv.response.alignment_transformation.data[14],srv.response.alignment_transformation.data[15];
  std::cout << transform(12) << transform(13) << transform(14) << transform(15);

  Eigen::Vector4f needle_startpoint;
  Eigen::Vector4f needle_goalpoint;
  needle_startpoint = {-30.0/1000, -100.674/1000, 160.0/1000, 1.0};
  needle_goalpoint = {-4.0/1000, -21.0/1000, 22.0/1000, 1.0};
  needle_startpoint = transform*needle_startpoint;
  needle_goalpoint = transform*needle_goalpoint;
  std::cout << "Start_pos: " << "\n" << "x: " << needle_startpoint(0) << "\n" << "y: " << needle_startpoint(1) << "\n" << "z: " << needle_startpoint(2) << "\n";
  std::cout << "Goal_pos: " << "\n" << "x: " << needle_goalpoint(0) << "\n" << "y: " << needle_goalpoint(1) << "\n" << "z: " << needle_goalpoint(2) << "\n";





    visualization_msgs::Marker points;
    points.header.frame_id = "rgb_camera_link";
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action  = visualization_msgs::Marker::ADD;
    points.pose.orientation.w =  1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.005;
    points.scale.y = 0.005;
    points.scale.z = 0.005;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;
    float x = needle_startpoint(0);
    float y = needle_startpoint(1);
    float z = needle_startpoint(2);
      geometry_msgs::Point p;
      p.x = x;
      p.y = y;
      p.z = z;
      points.points.push_back(p);

     x = needle_goalpoint(0);
     y = needle_goalpoint(1);
     z = needle_goalpoint(2);
    p.x = x;
    p.y = y;
    p.z = z;
    points.points.push_back(p);


    marker_pub.publish(points);

}