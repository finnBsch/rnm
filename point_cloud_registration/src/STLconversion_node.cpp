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

using namespace std::chrono_literals;
  pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
  {
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (255, 255, 255);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
  }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "STLconversion_node");
  ros::NodeHandle nodeHandle;
  ros::Publisher publisher;
  publisher = nodeHandle.advertise<sensor_msgs::PointCloud2> ("skeleton_cloud", 1);

  const std::string STL_file_name =  "/home/niklas/Documents/RNM/provided_data_scanning/Skeleton.stl";

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr skeleton_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PolygonMesh mesh;

  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  pcl::io::loadPolygonFileSTL( STL_file_name,mesh);
  pcl::io::mesh2vtk(mesh, polydata); //Convert PolygonMesh object into vtkPolyData object
  pcl::io::vtkPolyDataToPointCloud(polydata, *skeleton_cloud);


  for (int i = 0; i < skeleton_cloud->points.size(); i++)
  {
    skeleton_cloud->points[i].x /=1000 ;
    skeleton_cloud->points[i].y /=1000 ;
    skeleton_cloud->points[i].z /=1000 ;
    //skeleton_cloud_scaled->points.push_back(pcl::PointXYZRGB(pt.x * xStretch, pt.y * yStretch, pt.z * zStretch));
  }
  //SUBSAMPLE---------------------------------------------------------------------------------------------------------------
   //Initialize container for PC message, for the filtered cloud and pointer to PC message
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr skeleton_cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
  // Convert to PCL data type
  //pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud (skeleton_cloud);
  vg.setLeafSize (0.005, 0.005, 0.005);
  vg.filter (*skeleton_cloud);

  pcl::visualization::PCLVisualizer::Ptr viewer2;
      viewer2 = simpleVis(skeleton_cloud);


  sensor_msgs::PointCloud2 output;

  pcl::toROSMsg(*skeleton_cloud, output);
  output.header.frame_id = "point_link";
  publisher.publish (output);

  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer2->wasStopped ())
  {
    viewer2->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }
  ros::spin ();
  return 0;
}

//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());//Create a point cloud object
//pcl::PolygonMesh mesh;
//vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
//int a=pcl::io::loadPolygonFilePLY("D:\\02software\\points_resource\\horse.ply", mesh); //PCL uses VTK's IO interface to directly read stl, ply, obj, etc. Format the 3D point cloud data and pass it to the PolygonMesh object
//To read the stl file, use the function loadPolygonFileSTL
//pcl::io::mesh2vtk(mesh, polydata); //Convert PolygonMesh object into vtkPolyData object
//pcl::io::vtkPolyDataToPointCloud(polydata, *cloud);