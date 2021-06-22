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
#include <pcl/registration/ndt.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>


//using namespace std::chrono_literals;

class FeatureCloud
{
 public:
  // A bit of shorthand
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
  typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
  typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
  typedef pcl::search::KdTree<pcl::PointXYZRGB> SearchMethod;

  FeatureCloud () :
      search_method_xyz_ (new SearchMethod),
      normal_radius_ (0.025f),
      feature_radius_ (0.045f)
  {}

  ~FeatureCloud () {}

  // Process the given cloud
  void
  setInputCloud (PointCloud::Ptr xyz)
  {
    xyz_ = xyz;
    processInput ();
  }

  // Load and process the cloud in the given PCD file
  void
  loadInputCloud (const std::string &pcd_file)
  {
    xyz_ = PointCloud::Ptr (new PointCloud);
    pcl::io::loadPCDFile (pcd_file, *xyz_);
    processInput ();
  }

  // Get a pointer to the cloud 3D points
  PointCloud::Ptr
  getPointCloud () const
  {
    return (xyz_);
  }

  // Get a pointer to the cloud of 3D surface normals
  SurfaceNormals::Ptr
  getSurfaceNormals () const
  {
    return (normals_);
  }

  // Get a pointer to the cloud of feature descriptors
  LocalFeatures::Ptr
  getLocalFeatures () const
  {
    return (features_);
  }

 protected:
  // Compute the surface normals and local features
  void
  processInput ()
  {
    computeSurfaceNormals ();
    computeLocalFeatures ();
  }

  // Compute the surface normals
  void
  computeSurfaceNormals ()
  {
    normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
    norm_est.setInputCloud (xyz_);
    norm_est.setSearchMethod (search_method_xyz_);
    norm_est.setRadiusSearch (normal_radius_);
    norm_est.compute (*normals_);
  }

  // Compute the local feature descriptors
  void
  computeLocalFeatures ()
  {
    features_ = LocalFeatures::Ptr (new LocalFeatures);

    pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setInputCloud (xyz_);
    fpfh_est.setInputNormals (normals_);
    fpfh_est.setSearchMethod (search_method_xyz_);
    fpfh_est.setRadiusSearch (feature_radius_);
    fpfh_est.compute (*features_);
  }

 private:
  // Point cloud data
  PointCloud::Ptr xyz_;
  SurfaceNormals::Ptr normals_;
  LocalFeatures::Ptr features_;
  SearchMethod::Ptr search_method_xyz_;

  // Parameters
  float normal_radius_;
  float feature_radius_;
};


//---------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------

class TemplateAlignment
{
 public:

  // A struct for storing alignment results
  struct Result
  {
    float fitness_score;
    Eigen::Matrix4f final_transformation;
    //PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  TemplateAlignment () :
      min_sample_distance_ (0.05f),//0.05
      max_correspondence_distance_ (0.1f*0.1f), //0.01f*0.01f
      nr_iterations_ (300)
  {
    // Initialize the parameters in the Sample Consensus Initial Alignment (SAC-IA) algorithm
    sac_ia_.setMinSampleDistance (min_sample_distance_);
    sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
    sac_ia_.setMaximumIterations (nr_iterations_);
  }

  ~TemplateAlignment () {}

  // Set the given cloud as the target to which the templates will be aligned
  void
  setTargetCloud (FeatureCloud &target_cloud)
  {
    target_ = target_cloud;
    sac_ia_.setInputTarget (target_cloud.getPointCloud ());
    sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures ());
  }

  // Add the given cloud to the list of template clouds
  void
  addTemplateCloud (FeatureCloud &template_cloud)
  {
    templates_.push_back (template_cloud);
  }

  // Align the given template cloud to the target specified by setTargetCloud ()
  void
  align (FeatureCloud &template_cloud, TemplateAlignment::Result &result)
  {
    sac_ia_.setInputSource (template_cloud.getPointCloud ());
    sac_ia_.setSourceFeatures (template_cloud.getLocalFeatures ());

    pcl::PointCloud<pcl::PointXYZRGB> registration_output;
    sac_ia_.align (registration_output);

    result.fitness_score = (float) sac_ia_.getFitnessScore (max_correspondence_distance_);
    result.final_transformation = sac_ia_.getFinalTransformation ();
  }

  // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
  void
  alignAll (std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
  {
    results.resize (templates_.size ());
    for (std::size_t i = 0; i < templates_.size (); ++i)
    {
      align (templates_[i], results[i]);
    }
  }

  // Align all of template clouds to the target cloud to find the one with best alignment score
  int
  findBestAlignment (TemplateAlignment::Result &result)
  {
    // Align all of the templates to the target cloud
    std::vector<Result, Eigen::aligned_allocator<Result> > results;
    alignAll (results);

    // Find the template with the best (lowest) fitness score
    float lowest_score = std::numeric_limits<float>::infinity ();
    int best_template = 0;
    for (std::size_t i = 0; i < results.size (); ++i)
    {
      const Result &r = results[i];
      if (r.fitness_score < lowest_score)
      {
        lowest_score = r.fitness_score;
        best_template = (int) i;
      }
    }

    // Output the best alignment
    result = results[best_template];
    return (best_template);
  }

 private:
  // A list of template clouds and the target to which they will be aligned
  std::vector<FeatureCloud> templates_;
  FeatureCloud target_;

  // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
  pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33> sac_ia_;
  float min_sample_distance_;
  float max_correspondence_distance_;
  int nr_iterations_;
};

//---------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------
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

pcl::visualization::PCLVisualizer::Ptr simpleViscomb (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud2)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (255, 255, 255);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "sample cloud");
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud2, "template_cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "template_cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

//---------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------


int main(int argc, char** argv)
{
  ros::init(argc, argv, "STLconversion_node");
  ros::NodeHandle nodeHandle;
  ros::Publisher publisher;
  publisher = nodeHandle.advertise<sensor_msgs::PointCloud2> ("skeleton_cloud", 1);
  ros::Publisher publisher_stitch;
  publisher_stitch = nodeHandle.advertise<sensor_msgs::PointCloud2> ("stitched_cloud", 1);

  const std::string STL_file_name =  "/home/niklas/Documents/RNM/Scanning/Skeleton.stl";

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
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr skeleton_cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
  // Convert to PCL data type
  //pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud (skeleton_cloud);
  vg.setLeafSize (0.005, 0.005, 0.005);
  vg.filter (*skeleton_cloud);

  // transformation of skeleton cloud for trying different initial conditions
   float alpha = M_PI/4;
   float beta = M_PI/8;
   float gamma = M_PI/2;
   static Eigen::VectorXf offset(3);
   offset << -0.5, -0.2, -0.1;
   static Eigen::MatrixXf initial_tf(4,4);
   initial_tf << cos(beta)*cos(gamma), -cos(beta)*sin(gamma), sin(beta), offset(0),
       cos(alpha)*sin(gamma) + sin(alpha)*sin(beta)*cos(gamma), cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma), -sin(alpha)*cos(beta), offset(1),
       sin(alpha)*sin(gamma) - cos(alpha)*sin(beta)*cos(gamma), sin(alpha)*cos(gamma) + cos(alpha)*sin(beta)*sin(gamma), cos(alpha)*cos(beta), offset(2),
       0, 0, 0, 1;
//transformation
 // pcl::transformPointCloud (*skeleton_cloud, *skeleton_cloud, initial_tf);

  //pcl::visualization::PCLVisualizer::Ptr viewer2;
   //   viewer2 = simpleVis(skeleton_cloud);

// Load the object templates specified in the object_templates.txt file
  std::vector<FeatureCloud> object_templates;
  object_templates.resize (0);

/*  for (int i = 0; i < 10; ++i) {
    std::ostringstream oss;
    oss << "/home/konrad/Documents/RNM/skeleton_part" << i << ".pcd";
    std::string file = oss.str();

    FeatureCloud template_cloud;
    template_cloud.loadInputCloud(file);
    object_templates.push_back (template_cloud);
  }*/
  FeatureCloud template_cloud;
  template_cloud.setInputCloud(skeleton_cloud);
  object_templates.push_back (template_cloud);


  // load stitched point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr stitched_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::io::loadPCDFile( "/home/niklas/Documents/RNM/stitched_cloud.pcd", *stitched_cloud);

  vg.setInputCloud (stitched_cloud);
  vg.setLeafSize (0.005, 0.005, 0.005);
  vg.filter (*stitched_cloud);

 // pcl::visualization::PCLVisualizer::Ptr viewer3;
 // viewer3 = simpleVis(stitched_cloud);

  // Preprocess the cloud by...
  // ...removing distant points
  //sinn macht das noch nicht so
  //const float depth_limit = 1;
  //pcl::PassThrough<pcl::PointXYZRGB> pass;
  //pass.setInputCloud (stitched_cloud);
  //pass.setFilterFieldName ("z");
  //pass.setFilterLimits (0, depth_limit);
  //pass.filter (*stitched_cloud);

  /*
  // Assign to the target FeatureCloud
  FeatureCloud target_cloud;
  target_cloud.setInputCloud (stitched_cloud);



  // Set the TemplateAlignment inputs
  TemplateAlignment template_align;
  for (std::size_t i = 0; i < object_templates.size (); ++i)
  {
    template_align.addTemplateCloud (object_templates[i]);
  }
  template_align.setTargetCloud (target_cloud);

  // Find the best template alignment
  TemplateAlignment::Result best_alignment;
  int best_index = template_align.findBestAlignment (best_alignment);
  const FeatureCloud &best_template = object_templates[best_index];

  // Print the alignment fitness score (values less than 0.00002 are good)
  printf ("Best fitness score: %f\n", best_alignment.fitness_score);
  // Print the rotation matrix and translation vector
  Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3,3>(0, 0);
  Eigen::Vector3f translation = best_alignment.final_transformation.block<3,1>(0, 3);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::transformPointCloud (*best_template.getPointCloud (), *transformed_cloud, best_alignment.final_transformation);

*/

  // Types
  typedef pcl::PointXYZRGBNormal PointNT;
  typedef pcl::PointCloud<PointNT> PointCloudT;
  typedef pcl::FPFHSignature33 FeatureT;
  typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
  typedef pcl::PointCloud<FeatureT> FeatureCloudT;
  typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

  // Point clouds
  PointCloudT::Ptr object (new PointCloudT);
  PointCloudT::Ptr object_aligned (new PointCloudT);
  PointCloudT::Ptr scene (new PointCloudT);
  FeatureCloudT::Ptr object_features (new FeatureCloudT);
  FeatureCloudT::Ptr scene_features (new FeatureCloudT);

  pcl::io::loadPCDFile( "/home/niklas/Documents/RNM/stitched_cloud.pcd", *scene);

  pcl::io::vtkPolyDataToPointCloud(polydata, *object);

  for (int i = 0; i < object->points.size(); i++)
  {
    object->points[i].x /=1000 ;
    object->points[i].y /=1000 ;
    object->points[i].z /=1000 ;
    //skeleton_cloud_scaled->points.push_back(pcl::PointXYZRGB(pt.x * xStretch, pt.y * yStretch, pt.z * zStretch));
  }

  //transformation
   pcl::transformPointCloud(*object, *object, initial_tf);

  // Downsample
  pcl::console::print_highlight ("Downsampling...\n");
  pcl::VoxelGrid<PointNT> grid;
  const float leaf = 0.005f;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (object);
  grid.filter (*object);
  grid.setInputCloud (scene);
  grid.filter (*scene);

  pcl::console::print_highlight ("Estimating scene normals...\n");
  pcl::NormalEstimationOMP<PointNT,PointNT> nest;
  nest.setRadiusSearch (0.01);
  nest.setInputCloud (object);
  nest.compute (*object);
  nest.setInputCloud (scene);
  nest.compute (*scene);

  // Estimate features
  pcl::console::print_highlight ("Estimating features...\n");
  FeatureEstimationT fest;
  fest.setRadiusSearch (0.025);
  fest.setInputCloud (object);
  fest.setInputNormals (object);
  fest.compute (*object_features);
  fest.setInputCloud (scene);
  fest.setInputNormals (scene);
  fest.compute (*scene_features);

  // Perform alignment
  pcl::console::print_highlight ("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
  align.setInputSource (object);
  align.setSourceFeatures (object_features);
  align.setInputTarget (scene);
  align.setTargetFeatures (scene_features);
  align.setMaximumIterations (50000); // Number of RANSAC iterations
  align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (5); // Number of nearest features to use
  align.setSimilarityThreshold (0.8f); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
  align.setInlierFraction (0.5f); // Required inlier fraction for accepting a pose hypothesis

  {
    pcl::ScopeTime t("Alignment");
    align.align (*object_aligned);
  }

  if (align.hasConverged ())
  {

  /*
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_aligned_RGB (new pcl::PointCloud<pcl::PointXYZRGB>);
    object_aligned_RGB->resize(object_aligned->size());

    for (size_t i = 0; i < object_aligned->points.size(); ++i)
    {
      object_aligned_RGB->points[i].x=object_aligned->points[i].x; //error
      object_aligned_RGB->points[i].y=object_aligned->points[i].y; //error
      object_aligned_RGB->points[i].z=object_aligned->points[i].z; //error
      object_aligned_RGB->points[i].r=object_aligned->points[i].r; //error
      object_aligned_RGB->points[i].g=object_aligned->points[i].g; //error
      object_aligned_RGB->points[i].b=object_aligned->points[i].b; //error
    }
    */

/*
  pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB> ndt;
  //Set the NDT parameter dependent on the scale
  //Set the minimum conversion difference for the termination condition
  ndt.setTransformationEpsilon(0.01);//0.01
  //Set the maximum step size for More-Thuente line search
  ndt.setStepSize(0.1); //0.1
  //Set the resolution of the NDT grid structure (VoxelGridCovariance)
  ndt.setResolution(0.1); //1.0
  //Set the maximum number of matching iterations
  ndt.setMaximumIterations(10);
  // Set the point cloud to be registered
  ndt.setInputCloud(transformed_cloud);
  //Set point cloud registration target
  ndt.setInputTarget(stitched_cloud);
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  ndt.align (*transformed_cloud);
  pcl::transformPointCloud (*transformed_cloud, *transformed_cloud, ndt.getFinalTransformation());
*/



  pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
  icp.setInputSource(object_aligned);
  icp.setInputTarget(scene);

  //ICP parameters (examples), we still need to adjust them
  //icp.setEuclideanFitnessEpsilon(1);
  //icp.setTransformationEpsilon(1e-20);
  icp.setMaximumIterations(200);

  //align new cloud to stitched cloud and add to the total stitched cloud.
  icp.align(*object_aligned);



  //pcl::visualization::PCLVisualizer::Ptr viewer4;
  //viewer4 = simpleViscomb(stitched_cloud,skeleton_cloud);

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*object_aligned, output);
  output.header.frame_id = "rgb_camera_link";
  publisher.publish (output);

  sensor_msgs::PointCloud2 output_stitch;
  pcl::toROSMsg(*scene, output_stitch);
  output_stitch.header.frame_id = "rgb_camera_link";
  publisher_stitch.publish (output_stitch);

  }
  else
  {
    pcl::console::print_error ("Alignment failed!\n");
    return (1);
  }

  return (0);

//pcl::io::savePCDFile( "/home/niklas/Documents/RNM/skeleton_cloud.pcd", *skeleton_cloud, true ); // Binary format

//--------------------
  // -----Main loop-----
  //--------------------
 // while (!viewer2->wasStopped ())
 // {
  //  viewer2->spinOnce (100);
   // viewer3->spinOnce (100);
   // viewer4->spinOnce (100);
   // std::this_thread::sleep_for(100ms);
  //}


  //ros::spin ();
  //return 0;

}

//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());//Create a point cloud object
//pcl::PolygonMesh mesh;
//vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
//int a=pcl::io::loadPolygonFilePLY("D:\\02software\\points_resource\\horse.ply", mesh); //PCL uses VTK's IO interface to directly read stl, ply, obj, etc. Format the 3D point cloud data and pass it to the PolygonMesh object
//To read the stl file, use the function loadPolygonFileSTL
//pcl::io::mesh2vtk(mesh, polydata); //Convert PolygonMesh object into vtkPolyData object
//pcl::io::vtkPolyDataToPointCloud(polydata, *cloud);