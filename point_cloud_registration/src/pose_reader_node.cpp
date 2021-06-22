#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

void
save_joint_state (const sensor_msgs::JointStateConstPtr& joint_states)
{
  int c = getch();   // call your non-blocking input function
  if (c == 'g') {

    {
      joint_states->position[0], joint_states->position[1], joint_states->position[2],
          joint_states->position[3], joint_states->position[4], joint_states->position[5],
          joint_states->position[6]
    }
  }
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "VoxelGridFilter");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/joint_states", 1, save_joint_state);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("filtered_clouds", 1);

  // Spin
  ros::spin ();
}