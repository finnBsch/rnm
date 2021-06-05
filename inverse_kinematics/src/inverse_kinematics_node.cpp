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
#include <opencv2/core/core.hpp>
#include <opencv2/core_c.h>
#include <libfranka/franka/model.h>
#include <libfranka/franka/robot.h>
#include <libfranka/franka/robot_state.h>




using namespace tf;
using namespace Eigen;


class IncrementalInverseKinematics {

private:
    double incrementalStepSize;
    bool initializing = true;
    boost::array<double, 7> desired_joint_angles_;
    VectorXd joint_angles_;
    VectorXd current_transformationmatrix_vector_;
    VectorXd final_transformationmatrix_vector_;
    ros::ServiceClient* client_;
    sensor_msgs::JointState joint_state_msg;
    int size_;

public:
    std::string text = "vorher";
    explicit IncrementalInverseKinematics(ros::ServiceClient* client, int size): size_(size), client_(client),joint_angles_(7, 1), current_transformationmatrix_vector_(size, 1), final_transformationmatrix_vector_(size, 1){
        incrementalStepSize = 10.0;

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
    boost::array<double, 7> transformVectorToArray(VectorXd &vector) {
        return (boost::array<double, 7>){vector[0], vector[1], vector[2],
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
        forward_kin::get_endeffector srv;
        srv.request.joint_angles = transformVectorToArray(joint_angles_);

        auto a = client_->call(srv);
        if (a) {
            //ROS_INFO("A-Matrix: %f", srv.response.layout.data[0]);
            //ROS_INFO("THETA1: %f",joint_angles_(0));
        } else {
          //  ROS_INFO("THETA1: %f",joint_angles_(0));
            ROS_ERROR("Failed to call service forward_kin");
            exit; //TODO find better solution
        }
        // get the current A Matrix from the forward_kin node

        if(size_ ==12) {
            current_transformationmatrix_vector_
                    << srv.response.layout.data[0], srv.response.layout.data[4], srv.response.layout.data[8],
                    srv.response.layout.data[1], srv.response.layout.data[5], srv.response.layout.data[9],
                    srv.response.layout.data[2], srv.response.layout.data[6], srv.response.layout.data[10],
                    srv.response.layout.data[3], srv.response.layout.data[7], srv.response.layout.data[11];
        } else{
            current_transformationmatrix_vector_ << srv.response.end_effector_pos[0], srv.response.end_effector_pos[1],
                                                    srv.response.end_effector_pos[2], 0, 0, 0;
        }

        //---------------- DO INCREMENTAL CALCULATION ----------------
        if(initializing) {
            if (size_ == 12) {
            final_transformationmatrix_vector_
                    << 0.20077, -0.894, -0.40058, 0.40058, 0.44807, -0.79923, 0.894, 0, 0.44807, 0.21926, 0, 0.62827; // ONLY FOR TESTING
            } else{
                final_transformationmatrix_vector_<< 0,0,0,0,0,0;
            }
            initializing = false;
        }

        // calculate the difference
        VectorXd delta_A = (final_transformationmatrix_vector_ - current_transformationmatrix_vector_);
        ROS_INFO("Joint Angle %f",joint_angles_(2));
        if (abs(delta_A.maxCoeff()) <= 0.01) {
            ROS_INFO("I am finished: %f",delta_A(0));
            initializing = true;
            return transformVectorToArray(joint_angles_);
        } else {
            MatrixXd pinvJ;
            if (size_ == 12) {
            MatrixXd J = calculateJacobian(joint_angles_);
            //pinvJ = J.completeOrthogonalDecomposition().pseudoInverse();
            //pinvJ = pinv_eigen_based(J);
            cvInvert(&J, &pinvJ, 0);
            } else{
                MatrixXd J;
                //J = franka::Model::bodyJacobian(franka::Frame, joint_angles_);
            }

            // calculate new joint angles
            joint_angles_ = joint_angles_ + pinvJ * delta_A / incrementalStepSize;

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
        desired_joint_angles_ = incrementalStep();

        std::cout << desired_joint_angles_[0] << desired_joint_angles_[1];
        res.ik_jointAngles = {desired_joint_angles_[0], desired_joint_angles_[1],
                              desired_joint_angles_[2], desired_joint_angles_[3],
                              desired_joint_angles_[4], desired_joint_angles_[5], desired_joint_angles_[6]};
        return true;
    }

    Eigen::MatrixXd pinv_eigen_based(Eigen::MatrixXd & origin, const float er = 0) {
        // perform svd decomposition
        Eigen::JacobiSVD<Eigen::MatrixXd> svd_holder(origin,
                                                     Eigen::ComputeThinU |
                                                     Eigen::ComputeThinV);
        // Build SVD decomposition results
        Eigen::MatrixXd U = svd_holder.matrixU();
        Eigen::MatrixXd V = svd_holder.matrixV();
        Eigen::MatrixXd D = svd_holder.singularValues();

        // Build the S matrix
        Eigen::MatrixXd S(V.cols(), U.cols());
        S.setZero();

        for (unsigned int i = 0; i < D.size(); ++i) {

            if (D(i, 0) > er) {
                S(i, i) = 1 / D(i, 0);
            } else {
                S(i, i) = 0;
            }
        }

        // pinv_matrix = V * S * U^T
        return V * S * U.transpose();
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
    auto client = node_handle.serviceClient<forward_kin::get_endeffector>(
            "/forward_kin_node/get_endeffector");
    int size = 12;
    IncrementalInverseKinematics inverse_kinematics(&client, size);
    ros::ServiceServer service = node_handle.advertiseService("unserService",&IncrementalInverseKinematics::ik_jointAngles,&inverse_kinematics);
    // inverse_kinematics.startIncrementalInverseKinvematics();
    std::cout << inverse_kinematics.text;

    ros::spin();
}

