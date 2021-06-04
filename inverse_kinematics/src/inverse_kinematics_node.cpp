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
    boost::array<double, 7> desired_joint_angles_;
    VectorXd joint_angles_;
    VectorXd current_transformationmatrix_vector_;
    VectorXd final_transformationmatrix_vector_;
    ros::NodeHandle node_handle_;
    ros::ServiceClient client;
    sensor_msgs::JointState joint_state_msg;
    forward_kin::get_endeffector srv;

public:
    std::string text = "vorher";
    explicit IncrementalInverseKinematics(ros::NodeHandle node_handle): node_handle_(node_handle), joint_angles_(7, 1), current_transformationmatrix_vector_(12, 1), final_transformationmatrix_vector_(12, 1){
        incrementalStepSize = 10;
        client = node_handle_.serviceClient<forward_kin::get_endeffector>(
                "forward_kin_node/get_endeffector");
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
/*
    void startIncrementalInverseKinvematics(){
        std::cout << "START FUNCTION";
        service = node_handle_.advertiseService("unserService",ik_jointAngles);
    }
*/
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
    boost::array<double, 7> incrementalStep() {

        // --------------- GET JOINT ANGLES -------------------
        //joint_state_msg = *(ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", ros::Duration(10)));
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
        current_transformationmatrix_vector_
                << srv.response.layout.data[0], srv.response.layout.data[1], srv.response.layout.data[2], srv.response.layout.data[3],
                srv.response.layout.data[4], srv.response.layout.data[5], srv.response.layout.data[6], srv.response.layout.data[7],
                srv.response.layout.data[8], srv.response.layout.data[9], srv.response.layout.data[10], srv.response.layout.data[11],
                srv.response.layout.data[12], srv.response.layout.data[13], srv.response.layout.data[14], srv.response.layout.data[15];

        //---------------- DO INCREMENTAL CALCULATION ----------------

        // calculate the difference
        VectorXd delta_A = (final_transformationmatrix_vector_ - current_transformationmatrix_vector_);

        if (abs(delta_A.maxCoeff()) <= 0.01) {
            std::cout << "ich war in der if schleife";
            return transformVectorToArray(joint_angles_);
        } else {

            MatrixXd J = calculateJacobian(joint_angles_);
            MatrixXd pinvJ = J.completeOrthogonalDecomposition().pseudoInverse();

            // calculate new joint angles
            joint_angles_ = joint_angles_ + pinvJ * delta_A / incrementalStepSize;

            // call forward kin node and transmit the new joint angles
            srv.request.joint_angles = transformVectorToArray(joint_angles_);

            return incrementalStep();
        }
    }

    bool ik_jointAngles(inverse_kinematics::unserService::Request &req,
                        inverse_kinematics::unserService::Response &res) {
        text = "nachher";

        for(int i=0;i<7;i++) {
            joint_angles_(i) = req.initial_joint_angles[i];
        }
    //TODO goal pos in service
        final_transformationmatrix_vector_ = current_transformationmatrix_vector_ * 0.40; // ONLY FOR TESTING

        desired_joint_angles_ = incrementalStep();

        std::cout << desired_joint_angles_[0] << desired_joint_angles_[1];
        res.ik_jointAngles = {desired_joint_angles_[0], desired_joint_angles_[1],
                              desired_joint_angles_[2], desired_joint_angles_[3],
                              desired_joint_angles_[4], desired_joint_angles_[5], desired_joint_angles_[6]};
        std::cout << "2222222";
    }

};

int main(int argc, char** argv)  {
  // create inverse-kinematics node
  ros::init(argc,argv, "inverse_kinematics_node");
  ros::NodeHandle node_handle("~");
    std::cout << "moin moin";

  //-------------------- Joint Angles ---------------------------
  // Parse parameters specified in the .launch file
  std::string topic_name;
  int queue_size;
  node_handle.getParam("topic_name", topic_name);
  node_handle.getParam("queue_size", queue_size);
    IncrementalInverseKinematics inverse_kinematics(node_handle);
    ros::ServiceServer service = node_handle.advertiseService("unserService",&IncrementalInverseKinematics::ik_jointAngles,&inverse_kinematics);
   // inverse_kinematics.startIncrementalInverseKinvematics();
    std::cout << inverse_kinematics.text;

  ros::spin();
}

