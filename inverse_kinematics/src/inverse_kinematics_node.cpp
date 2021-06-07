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
//#include "forward_kin/get_endeffector.h"
#include "jacobian.h"
#include <eigen_conversions/eigen_msg.h>
#include "inverse_kinematics/unserService.h"
#include <opencv2/core/core.hpp>


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
    int counter_;
    int limit_;

    VectorXd a;
    VectorXd d;
    VectorXd alpha;
    MatrixXd A_total;
    VectorXd vector;
    MatrixXd ret_mat;
    std::array<MatrixXd, 8> a_;
    MatrixXd J;
    MatrixXd pinvJ;
    VectorXd delta_A;
    VectorXd deltaQ;

public:
    std::string text = "vorher";
    /*IncrementalInverseKinematics(ros::ServiceClient* client, int size): size_(size), client_(client),joint_angles_(7, 1), current_transformationmatrix_vector_(size, 1), final_transformationmatrix_vector_(size, 1){
        incrementalStepSize = 100000.0;
        limit_ = 10000;
        counter_ = 0;
    }*/

    explicit IncrementalInverseKinematics(int size): size_(size),joint_angles_(7, 1), current_transformationmatrix_vector_(size, 1), final_transformationmatrix_vector_(size, 1), a(8), d(8), alpha(8),vector(12,1),A_total(4,4),ret_mat(4,4){
        incrementalStepSize = 10000.0;
        limit_ = 10000;
        counter_ = 0;
        client_ = nullptr;
        a << 0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0;
        d << 0.333, 0, 0.316, 0, 0.384, 0, 0, 0.107;
        alpha << 0, -M_PI/2, M_PI/2, M_PI/2, -M_PI/2, M_PI/2, M_PI/2, 0;
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

    //__________________________________________BEGIN OF FORWARD

    MatrixXd get_transformationmatrix(const float theta, const float a, const float d, const float alpha){
        ret_mat << cos(theta), -sin(theta), 0, a,
                sin(theta) * cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d * sin(alpha),
                sin(theta) * sin(alpha), cos(theta)*sin(alpha), cos(alpha), d * cos(alpha),
                0, 0, 0, 1;
        return ret_mat;
    }

    VectorXd get_transformation_Vector() {
        for(int i  = 0; i<7; i++){
            a_.at(i) = get_transformationmatrix(joint_angles_(i), a(i), d(i), alpha(i));
        }
        a_.at(7) = get_transformationmatrix(0, a(7), d(7), alpha(7));
        //VectorXd in(4);
        //in << 0, 0, 0, 1;
        A_total=a_.at(0)*a_.at(1)*a_.at(2)*a_.at(3)*a_.at(4)*a_.at(5)*a_.at(6)*a_.at(7);
        //VectorXd out = A_total*in;


        vector<< A_total(0,0), A_total(1,0), A_total(2,0),
                A_total(0,1), A_total(1,1), A_total(2,1),
                A_total(0,2), A_total(1,2), A_total(2,2),
                A_total(0,3), A_total(1,3), A_total(2,3);

        return vector;
    }

    //_______________________________________________ END OF FORWARD





/*
 * This recursive function performs the incremental inverse kinematics. It takes the current joint angles, the current A and final A Matrix as 12x1 vectors. It returns the desired joint angles for the desired position of the robot
 *
 * @param: VectorXd thetas, VectorXd currentA. VectorXd finalA
 * returns: VectorXd desiredThetas
 */
    boost::array<double, 7> incrementalStep() {

        // FOR TESTING
        counter_ = counter_ + 1;

        //forward_kin::get_endeffector srv; $$$$$$$$$$$
        //srv.request.joint_angles = transformVectorToArray(joint_angles_);

        /* //ROUND JOINT ANGLES
        for(int i=0;i<7;i++){
            joint_angles_(i)= roundf(joint_angles_(i)*1000)/1000;
        }
        */




        // CONNECTION CHECK
        /*auto a = client_->call(srv);
        if (a) {
            //ROS_INFO("A-Matrix: %f", srv.response.layout.data[0]);
            //ROS_INFO("THETA1: %f",joint_angles_(0));
        } else {
          //  ROS_INFO("THETA1: %f",joint_angles_(0));
            ROS_ERROR("Failed to call service forward_kin");
            exit; //TODO find better solution
        }*/


        // GET CURRENT TRANSFORMATION MATRIX
        if(size_ ==12) {
            current_transformationmatrix_vector_ << get_transformation_Vector();
            /*
                    << srv.response.layout.data[0], srv.response.layout.data[4], srv.response.layout.data[8],
                    srv.response.layout.data[1], srv.response.layout.data[5], srv.response.layout.data[9],
                    srv.response.layout.data[2], srv.response.layout.data[6], srv.response.layout.data[10],
                    srv.response.layout.data[3], srv.response.layout.data[7], srv.response.layout.data[11];
            */

        } else{
            //current_transformationmatrix_vector_ << srv.response.end_effector_pos[0], srv.response.end_effector_pos[1],
              //                                      srv.response.end_effector_pos[2];
        }

        //---------------- DO INCREMENTAL CALCULATION ----------------

        // FOR TESTING, STARTING VECTOR
        if(initializing) {
            if (size_ == 12) {
            final_transformationmatrix_vector_
                    << 0.244948, 0.076194, 0.966538, 0.250202, 0.96417, -0.123872, -0.234584, 0.444379, 0.101854, 0.989368, -0.103804, 0.6328920001; // ONLY FOR TESTING
                    //<< 0.2760,-0.8509,   0.4470, -0.4470, -0.5253, -0.7240, 0.8509, 0, -0.5253, 0.3670, 0.8315,0;

            } else{
                final_transformationmatrix_vector_ << 0.5,0.5,0.5;
            }
            initializing = false;
        }


        // DIFFERENCE CALCULATION
         delta_A = final_transformationmatrix_vector_ - current_transformationmatrix_vector_;
        //output
        ROS_INFO("counter: %i", counter_);
        for(int i=0; i<current_transformationmatrix_vector_.rows();i++){
            ROS_INFO("current A: %f",current_transformationmatrix_vector_(i));
        }
        ROS_INFO("--------------------------");
        // Calculate the absolute of the delta_A Vector
        VectorXd abs_delta_A(size_,1);
        for (int i=0;i<size_;i++){
            abs_delta_A(i) = abs(delta_A(i));
        }

        if (abs_delta_A.maxCoeff() <= 0.1 ) {//|| counter_ == limit_) {
            for(int i=0; i<current_transformationmatrix_vector_.rows();i++){
                ROS_INFO("A_Matrix %f",current_transformationmatrix_vector_(i));
            }
            if(counter_ == limit_) {
               // ROS_ERROR("Limit ueberstritten: %i", counter_);
            } else {
                ROS_INFO("FINISHED");
            }
            return transformVectorToArray(joint_angles_);

        } else {

            // CALCULATE PSEODOINVERSE OF JACOBIAN
                //TODO include if statement for size_ in calculateJacobian
                J = calculateJacobian(joint_angles_, size_);
                pinvJ = J.completeOrthogonalDecomposition().pseudoInverse();
                //pinvJ = pinv_eigen_based(J);
                // MatrixXd JM= J*J.transpose();
                //pinvJ =  J.transpose() * JM.inverse() ;
                //cvInvert(&J, &pinvJ, 0);
                //J = franka::Model::bodyJacobian(franka::Frame, joint_angles_);

            //CALCULATE CHANGE IN JOINT ANGLES
             deltaQ = pinvJ * delta_A;
            //output change
            /*for(int i=0; i<deltaQ.rows();i++){
                ROS_INFO("deltaQ %f",deltaQ(i));
            }*/

            joint_angles_ = joint_angles_ + deltaQ / incrementalStepSize;
            /*for (int i=0; i< joint_angles_.rows();i++){
                ROS_INFO("Angles: %f", joint_angles_(i));
            }*/
            return incrementalStep();
        }
    }

    bool ik_jointAngles(inverse_kinematics::unserService::Request &req,
                        inverse_kinematics::unserService::Response &res) {

        // FOR TESTING LATER DELETE
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
/*
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
    }*/
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
    //auto client = node_handle.serviceClient<forward_kin::get_endeffector>(
      //      "/forward_kin_node/get_endeffector"); $$$$$$$$$$$$$$$
    int size = 12;
    //IncrementalInverseKinematics inverse_kinematics(&client, size);
    IncrementalInverseKinematics inverse_kinematics(size);
    ros::ServiceServer service = node_handle.advertiseService("unserService",&IncrementalInverseKinematics::ik_jointAngles,&inverse_kinematics);
    // inverse_kinematics.startIncrementalInverseKinvematics();
    std::cout << inverse_kinematics.text;

    ros::spin();
}
