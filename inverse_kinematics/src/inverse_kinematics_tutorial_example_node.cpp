/*
 * This is the example of Tutorial 5 to test and understand incremental inverse kinematics
 * author: Sean Maroofi
 * last edited: 25.5.21
 */
#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include <eigen3/Eigen/Dense>

#define PI 3.14159265

using namespace Eigen;

// matrices and vectors for the equations
static VectorXd initialTheta(3);
static Matrix4Xd initialA;
static Matrix4Xd desiredA;

// length of robot arm
int l1 = 100;
int l2 = 50;
int l3 = 150;

/*
 * This function returns the A Matrix of the example of Tutorial 5. It takes 3 joint angles as input parameters
 *
 * @param: float theta1, float theta2, float d3*
 * returns: Matrix A
 */
MatrixXd exampleMatrix_A_03(float theta1, float theta2, float d3){
  MatrixXd A(4,4);
  A <<  cos(theta1)*cos(theta2), -sin(theta1), cos(theta1)*sin(theta2), (l2 + l3 + d3)*cos(theta1)*sin(theta2),
        cos(theta2)*sin(theta1), cos(theta1) , sin(theta1)*sin(theta2), (l2 + l3 + d3)*sin(theta1)*sin(theta2),
        -sin(theta2)           , 0           , cos(theta2)            , l1 + (l2 + l3 + d3)*cos(theta2)       ,
        0                      , 0           , 0                      , 1;

  return A;
}

/*
 * This function takes A 4x4 Matrix and converts it to a 1x12 vector
 *
 * @param: MatrixXd M
 * returns: VectorXd v
 */
VectorXd convert4DMatrixTo12DVector(MatrixXd M){
  VectorXd v(12);
    v << M(0,0), M(1,0), M(2,0),
         M(0,1), M(1,1), M(2,1),
         M(0,2), M(1,2), M(2,2),
         M(0,3), M(1,3), M(2,3);
  return v;
}

/*
void callback(const sensor_msgs::JointState& msg) {
  std::array<MatrixXd, 8> a_;
  for(int i  = 0; i<8; i++){
    a_.at(i) = exampleMatrix_A_03(msg.position[i], a(i), d(i), alpha(i));
  }
  VectorXd in(4);
  in << 0, 0, 0, 1;
  VectorXd out = a_.at(0)*a_.at(1)*a_.at(2)*a_.at(3)*a_.at(4)*a_.at(5)*a_.at(6)*a_.at(7)*in;
  std::cout << "End_pos: " << "\n" << "x: " << out(0) << "\n" << "y: " << out(1) << "\n" << "z: " << out(2) << "\n";
}
*/

int main(int argc, char** argv)  {
  // initialize variables
  initialTheta << 40*PI/180, 40*PI/180, 20;
  initialA = exampleMatrix_A_03(initialTheta[0],initialTheta[1], initialTheta[2]);
  desiredA << 0.5792, -0.7071, 0.4056,  93.2834,
              0.5792,  0.7071, 0.4056,  93.2834,
             -0.5736,  0     , 0.8192, 288.4050,
              0     ,  0     , 0     ,   1;

  // Matrix to vectors transformation
  VectorXd initialA_vector = convert4DMatrixTo12DVector(initialA);
  VectorXd desiredA_vector = convert4DMatrixTo12DVector(desiredA);


  /*a << 0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0;
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
  ros::Subscriber sub = nh.subscribe("/joint_states", 10, callback);

  // Prevent the program from terminating
  // Blocks but still allows messages to be received
  ros::spin();*/
}