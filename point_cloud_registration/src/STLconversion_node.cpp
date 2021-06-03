#include <ros/ros.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/visualization/pcl_visualizer.h>

  //const std::string meshFileName = "~/Documents/RNM/provided_data_scanning/Skeleton.stl";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "STLconversion_node");
  ros::NodeHandle nodeHandle;
  ros::Publisher publisher;
  publisher = nodeHandle.advertise<sensor_msgs::PointCloud2> ("skeleton_cloud", 1);

  const std::string STL_file_name =  "/home/niklas/Documents/RNM/provided_data_scanning/Skeleton.stl";

  pcl::PointCloud<pcl::PointXYZ>::Ptr skeleton_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PolygonMesh mesh;

  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  pcl::io::loadPolygonFileSTL( STL_file_name,mesh);
  pcl::io::mesh2vtk(mesh, polydata); //Convert PolygonMesh object into vtkPolyData object
  pcl::io::vtkPolyDataToPointCloud(polydata, *skeleton_cloud);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("The name"));
  viewer->initCameraParameters();
  int v1(0), v2(1);
  viewer->createViewPort(0.0, 0.0, 100, 100, v1);//xmin,ymin,xmax,ymax
  viewer->createViewPort(0.5, 0,  100,    100, v2);
  viewer->setBackgroundColor(0, 1, 0, v1);
  viewer->setBackgroundColor(0, 0, 1, v2);
  viewer->createViewPortCamera(v1);//Create a new camera to make the view operation of the two windows independent
  //viewer->createViewPortCamera(v2);
  viewer->addPolygonMesh(mesh, "mesh",v1);
  viewer->addPointCloud(skeleton_cloud, "points",v2);
  viewer->addCoordinateSystem(1);

  sensor_msgs::PointCloud2 output;

  pcl::toROSMsg(*skeleton_cloud, output);
  publisher.publish (output);
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