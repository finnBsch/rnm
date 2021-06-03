/*
 * This is the example of Tutorial 5 to test and understand incremental inverse kinematics
 * author: Sean Maroofi
 * added: 29.5.21
 * last edited: 29.5.21
 */

#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include <eigen3/Eigen/Dense>
#include <stdlib.h>
#include "forward_kin/get_endeffector.h"
#include "jacobian.h"
#include <eigen_conversions/eigen_msg.h>
#include "inverse_kinematics/unserService.h"

using namespace tf;
using namespace Eigen;


class IncrementalInverseKinematics {

private:
    float incrementalStepSize;
    boost::array<double, 7> desiredJointAngles;

    VectorXd jointAngles;

    VectorXd currentA_vector;

    VectorXd finalA_vector;

    ros::NodeHandle node_handle_;
public:
    explicit IncrementalInverseKinematics(ros::NodeHandle node_handle): node_handle_(node_handle), jointAngles(7,1), currentA_vector(12,1), finalA_vector(12,1){
        incrementalStepSize = 10;

    }

/*
 * This function takes A 4x4 Matrix and converts it to a 1x12 vector
 *
 * @param: MatrixXd M
 * returns: VectorXd v
 */
    VectorXd convert4DMatrixTo12DVector(MatrixXd M) {
        VectorXd v(12);
        v << M(0, 0), M(1, 0), M(2, 0),
                M(0, 1), M(1, 1), M(2, 1),
                M(0, 2), M(1, 2), M(2, 2),
                M(0, 3), M(1, 3), M(2, 3);
        return v;
    }

    boost::array<double, 7> transformVectorToArray(VectorXd vector) {
        return {vector[0], vector[1], vector[2],
                vector[3], vector[4], vector[5],
                vector[6]};
    }

/*
 * This recursive function performs the incremental inverse kinematics. It takes the current joint angles, the current A and final A Matrix as 12x1 vectors. It returns the desired joint angles for the desired position of the robot
 *
 * @param: VectorXd thetas, VectorXd currentA. VectorXd finalA
 * returns: VectorXd desiredThetas
 */
    boost::array<double, 7> incrementalStep(sensor_msgs::JointState joint_state_msg, forward_kin::get_endeffector srv,
                                            ros::ServiceClient client) {

        // --------------- GET JOINT ANGLES -------------------
        joint_state_msg = *(ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", ros::Duration(10)));
        jointAngles
                << joint_state_msg.position[0], joint_state_msg.position[1], joint_state_msg.position[2], joint_state_msg.position[3], joint_state_msg.position[4], joint_state_msg.position[5], joint_state_msg.position[6];

        // --------------- GET A MATRIX -------------------
        // check connection
        auto a = client.call(srv);
        if (a) {
            ROS_INFO("A-Matrix: %f", srv.response.layout.data[0]);
        } else {
            ROS_ERROR("Failed to call service forward_kin");
            exit; //TODO find better solution
        }
        // get the current A Matrix from the forward_kin node
        currentA_vector
                << srv.response.layout.data[0], srv.response.layout.data[1], srv.response.layout.data[2], srv.response.layout.data[3],
                srv.response.layout.data[4], srv.response.layout.data[5], srv.response.layout.data[6], srv.response.layout.data[7],
                srv.response.layout.data[8], srv.response.layout.data[9], srv.response.layout.data[10], srv.response.layout.data[11],
                srv.response.layout.data[12], srv.response.layout.data[13], srv.response.layout.data[14], srv.response.layout.data[15];

        //---------------- DO INCREMENTAL CALCULATION ----------------

        // calculate the difference
        VectorXd delta_A = (finalA_vector - currentA_vector);

        if (abs(delta_A.maxCoeff()) <= 0.01) {
            return transformVectorToArray(jointAngles);
        } else {

            MatrixXd J = calculateJacobian(jointAngles);
            MatrixXd pinvJ = J.completeOrthogonalDecomposition().pseudoInverse();

            // calculate new joint angles
            VectorXd newJointAngles = jointAngles + pinvJ * delta_A / incrementalStepSize;

            // call forward kin node and transmit the new joint angles
            srv.request.joint_angles = transformVectorToArray(jointAngles);

            return incrementalStep(joint_state_msg, srv, client);
        }
    }

    bool ik_jointAngles(inverse_kinematics::unserService::Request &req,
                        inverse_kinematics::unserService::Response &res) {

        sensor_msgs::JointState joint_state_msg;

        // create client and subscribe to service
        forward_kin::get_endeffector srv;
        ros::ServiceClient client = nodeHandle.serviceClient<forward_kin::get_endeffector>(
                "forward_kin_node/get_endeffector");

        finalA_vector = currentA_vector * 0.80; // ONLY FOR TESTING

        desiredJointAngles = incrementalStep(joint_state_msg, srv, client);

        std::cout << desiredJointAngles[0] << desiredJointAngles[1];
        res.ik_jointAngles = {desiredJointAngles[0], desiredJointAngles[1],
                              desiredJointAngles[2], desiredJointAngles[3],
                              desiredJointAngles[4], desiredJointAngles[5], desiredJointAngles[6]};

    }

};

int main(int argc, char** argv)  {
  // create inverse-kinematics node
  ros::init(argc,argv, "inverse_kinematics_node");
  ros::NodeHandle node_handle("~");


  //-------------------- Joint Angles ---------------------------
  // Parse parameters specified in the .launch file
  std::string topic_name;
  int queue_size;
  node_handle.getParam("topic_name", topic_name);
  node_handle.getParam("queue_size", queue_size);
    IncrementalInverseKinematics inverse_kinematics(node_handle);
    //PUBLISH
    ros::ServiceServer service = node_handle.advertiseService("unserService", inverse_kinematics.ik_jointAngles);

  ros::spin();
}

