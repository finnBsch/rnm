#include <ros/ros.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Core>
#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>


int main(int argc, char** argv) {
  ros::init(argc, argv, "skeleton_cloud_splitter");
  ros::NodeHandle nodeHandle;

  ros::Publisher publisher;
  publisher = nodeHandle.advertise<sensor_msgs::PointCloud2>("skeleton_cloud", 1);

  const std::string STL_file_name = "/home/konrad/Documents/RNM/Scanning/Skeleton.stl";

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr skeleton_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PolygonMesh mesh;

  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  pcl::io::loadPolygonFileSTL(STL_file_name, mesh);
  pcl::io::mesh2vtk(mesh, polydata);  // Convert PolygonMesh object into vtkPolyData object
  pcl::io::vtkPolyDataToPointCloud(polydata, *skeleton_cloud);

  for (int i = 0; i < skeleton_cloud->points.size(); i++) {
    skeleton_cloud->points[i].x /= 1000;
    skeleton_cloud->points[i].y /= 1000;
    skeleton_cloud->points[i].z /= 1000;
  }

  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud(skeleton_cloud);
  vg.setLeafSize(0.002, 0.002, 0.002);
  vg.filter(*skeleton_cloud);

/*
  pcl::io::loadPCDFile( "/home/konrad/Documents/RNM/skeleton_part1.pcd", *skeleton_cloud);

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*skeleton_cloud, output);
  output.header.frame_id = "rgb_camera_link";
  publisher.publish(output);
*/

  // approixmate limits xyz
  // x +14 -20 split in the middle
  // y +30 -42 (-42)-(-13), (-13)-(+5), (+5)-(+30)
  // z +12 -12 split in the middle

  Eigen::VectorXf x_min(10);
  Eigen::VectorXf x_max(10);
  Eigen::VectorXf y_min(10);
  Eigen::VectorXf y_max(10);
  Eigen::VectorXf z_min(10);
  Eigen::VectorXf z_max(10);

  x_min << -0.2,  -0.2,  -0.2,  -0.2,  -0.2,  -0.03, -0.03, -0.03, -0.03, -0.03;
  x_max << -0.03, -0.03, -0.03, -0.03, -0.03,  0.14,  0.14,  0.14,  0.14,  0.14;
  y_min << -0.42, -0.13, -0.13, 0.05, 0.05, -0.42, -0.13, -0.13, 0.05, 0.05;
  y_max << -0.13, 0.05, 0.05, 0.3, 0.3, -0.13, 0.05, 0.05, 0.3, 0.3;
  z_min << -0.12, -0.12, 0.0, -0.12, 0.0, -0.12, -0.12, 0.0, -0.12, 0.0;
  z_max <<  0.12, 0.0, 0.12, 0.0, 0.12,  0.12, 0.0, 0.12, 0.0, 0.12;


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::CropBox<pcl::PointXYZRGB> crop;
  crop.setInputCloud(skeleton_cloud);


  for (int i = 0; i < 10; ++i) {
    Eigen::Vector4f min = {x_min(i), y_min(i), z_min(i), 1.0};
    Eigen::Vector4f max = {x_max(i), y_max(i), z_max(i), 1.0};
    crop.setMin(min);
    crop.setMax(max);
    crop.filter(*cropped_cloud);
    std::ostringstream oss;
    oss << "/home/konrad/Documents/RNM/skeleton_part" << i << ".pcd";
    std::string file = oss.str();
    pcl::io::savePCDFile(file, *cropped_cloud, true);
  }
  ros::spin();
}
