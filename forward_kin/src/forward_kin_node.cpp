#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include <eigen3/Eigen/Dense>
#include "forward_kin/get_endeffector.h"
#include "../../general/convert.h"
#include <std_msgs/Float64MultiArray.h>
#include "tf/transform_datatypes.h"
#include <eigen_conversions/eigen_msg.h>

using namespace std_msgs;
using namespace tf;
using namespace Eigen;

static VectorXd a(8);
static VectorXd d(8);
static VectorXd alpha(8);

MatrixXd get_transformationmatrix(const float theta, const float a, const float d, const float alpha){
  MatrixXd ret_mat(4,4);
  ret_mat << cos(theta), -sin(theta), 0, a,
      sin(theta) * cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d * sin(alpha),
      sin(theta) * sin(alpha), cos(theta)*sin(alpha), cos(alpha), d * cos(alpha),
      0, 0, 0, 1;
  return ret_mat;
}

bool get_end_effector(forward_kin::get_endeffector::Request  &req,
                      forward_kin::get_endeffector::Response &res) {
  std::array<MatrixXd, 8> a_;
  for(int i  = 0; i<7; i++){
    a_.at(i) = get_transformationmatrix(req.joint_angles[i], a(i), d(i), alpha(i));
  }
  a_.at(7) = get_transformationmatrix(0, a(7), d(7), alpha(7));
  VectorXd in(4);
  in << 0, 0, 0, 1;
  MatrixXd A_total=a_.at(0)*a_.at(1)*a_.at(2)*a_.at(3)*a_.at(4)*a_.at(5)*a_.at(6)*a_.at(7);
  VectorXd out = A_total*in;
  res.end_effector_pos =  {out[0], out[1], out[2]};
  //std::cout << "End_pos: " << "\n" << "x: " << out(0) << "\n" << "y: " << out(1) << "\n" << "z: " << out(2) << "\n";
  matrixEigenToMsg(A_total,res.layout);
  std::cout << res.layout;
  return true;
}

int main(int argc, char** argv)  {
  a << 0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0;
  d << 0.333, 0, 0.316, 0, 0.384, 0, 0, 0.107;
  alpha << 0, -M_PI/2, M_PI/2, M_PI/2, -M_PI/2, M_PI/2, M_PI/2, 0;
  ros::init(argc, argv, "forward_kin_node");

  // Get a node handle to the private parameters
  // e.g. /node_name/parameter_name can then be
  // read by nh.getParam("parameter_name", value);
  ros::NodeHandle  nh("~");
  // Parse parameters specified in the .launch file
  std::string topic_name;
  int queue_size;
  nh.getParam("topic_name", topic_name);
  nh.getParam("queue_size", queue_size);

  // Register a callback function (a function that is called every time a new message arrives)
  ros::ServiceServer service = nh.advertiseService("get_endeffector", get_end_effector);
  ros::spin();
}