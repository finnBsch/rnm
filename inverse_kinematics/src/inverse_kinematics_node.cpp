/*
 * This is the example of Tutorial 5 to test and understand incremental inverse kinematics
 * author: Sean Maroofi
 * added: 29.5.21
 * last edited: 9.6.21
 *
 *
 * WORKING FOR PARAMS:
 *
 * initial joint angles : -1.04, 0.32, 2.43, -1.97, -1.67119, 1.45099, 0.321082
 * finial A (12x1 order): < 0.244948, 0.96417, 0.101854, 0.076194, -0.123872, 0.989368, 0.966538, -0.234584, -0.103804, 0.250202, 0.444379, 0.632892;
 * incrementalStepSize: 0.00001
 * abort value: 0.05
 * no rounding
 * setZero() before their initialization: current_transformationmatrix_vector_, delta_A, abs_delta_A
 * pinvJ calculated with pinvJ = J.completeOrthogonalDecomposition().pseudoInverse()
 *
 *
 *
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
    Matrix4d A_total;
    VectorXd vector;
    Matrix4d ret_mat;
    std::array<MatrixXd, 8> a_;
    MatrixXd J;
    MatrixXd pinvJ;
    VectorXd delta_A;
    VectorXd deltaQ;
    VectorXd abs_delta_A;

public:
    std::string text = "vorher";

    // CONTRUCTOR FOR SERVICE CONNECTION WITH FORWARD KIN
    /*IncrementalInverseKinematics(ros::ServiceClient* client, int size): size_(size), client_(client),joint_angles_(7, 1), current_transformationmatrix_vector_(size, 1), final_transformationmatrix_vector_(size, 1){
        incrementalStepSize = 100000.0;
        limit_ = 10000;
        counter_ = 0;
    }*/

    // CONTRUCTOR FOR INVERSE KINEMATICS WITHOUT FORWARD KIN SERVICE CONNECTION
    explicit IncrementalInverseKinematics(int size): size_(size),joint_angles_(7, 1), current_transformationmatrix_vector_(size, 1), final_transformationmatrix_vector_(size, 1), a(8), d(8), alpha(8),vector(12,1),A_total(4,4),ret_mat(4,4), abs_delta_A(size_,1){
        incrementalStepSize = 0.00001;
        limit_ = 10000;
        counter_ = 0;
        client_ = nullptr;
        a << 0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0;
        d << 0.333, 0, 0.316, 0, 0.384, 0, 0, 0.107;
        alpha << 0, -M_PI/2, M_PI/2, M_PI/2, -M_PI/2, M_PI/2, M_PI/2, 0;
    }




    /*
 * This function takes a 4x4 Matrix and converts it to a 1x12 vector
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
 * This function takes a 7x1 Vector and converts it into an array
 *
 * @param: VectorXd v
 * returns: <double,7> array
 */
    boost::array<double, 7> transformVectorToArray(VectorXd &vector) {
        return (boost::array<double, 7>){vector[0], vector[1], vector[2],
                                         vector[3], vector[4], vector[5],
                                         vector[6]};
    }

    // ------------------------- FORWARD KINEMATICS -----------------------------


    /*
* This function return the 4x4 Transformationmatrix for the given Denavit Hagen Parameters
*
* @param: float theta, float a, float d, float alpha
* returns: MatrixXd A
*/
    MatrixXd get_transformationmatrix(const float theta, const float a, const float d, const float alpha){
        ret_mat << cos(theta), -sin(theta), 0, a,
                sin(theta) * cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d * sin(alpha),
                sin(theta) * sin(alpha), cos(theta)*sin(alpha), cos(alpha), d * cos(alpha),
                0, 0, 0, 1;
        return ret_mat;
    }


    /*
* This function performs the Forward kinematics and returns the overall transformation matrix to the goal position as a vector
*
* @param: -
* returns: VectorXd v
*/
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

    //END OF FORWARD





/*
 * This recursive function performs the incremental inverse kinematics. It returns the desired joint angles for the desired position of the robot
 *
 * @param: -
 * returns: VectorXd desiredThetas
 */
    boost::array<double, 7> incrementalStep() {

        // FOR TESTING
        counter_ = counter_ + 1;

        // ------------- FOR SERVICE CONNECTION ------------------------------
        //forward_kin::get_endeffector srv; $$$$$$$$$$$
        //srv.request.joint_angles = transformVectorToArray(joint_angles_);

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
        // -------------------------------------------------------------------


        // GET CURRENT TRANSFORMATION MATRIX
        if(size_ ==12) {
            //current_transformationmatrix_vector_.setZero();
            current_transformationmatrix_vector_ << get_transformation_Vector();

            // ------------- FOR SERVICE CONNECTION ------------------------------
            /*
                    << srv.response.layout.data[0], srv.response.layout.data[4], srv.response.layout.data[8],
                    srv.response.layout.data[1], srv.response.layout.data[5], srv.response.layout.data[9],
                    srv.response.layout.data[2], srv.response.layout.data[6], srv.response.layout.data[10],
                    srv.response.layout.data[3], srv.response.layout.data[7], srv.response.layout.data[11];
            */
            // -------------------------------------------------------------------
        } else{
            //current_transformationmatrix_vector_ << srv.response.end_effector_pos[0], srv.response.end_effector_pos[1],
              //                                      srv.response.end_effector_pos[2];
        }

        //---------------- DO INCREMENTAL CALCULATION ----------------

        // FOR TESTING, STARTING VECTOR
        if(initializing) {
            if (size_ == 12) {
            final_transformationmatrix_vector_
                    << 0.244948, 0.96417, 0.101854,
                       0.076194, -0.123872, 0.989368,
                       0.966538, -0.234584, -0.103804,
                       0.250202, 0.444379, 0.632892;      // ONLY FOR TESTING
                    //<< 0.2760,-0.8509,   0.4470, -0.4470, -0.5253, -0.7240, 0.8509, 0, -0.5253, 0.3670, 0.8315,0;

            } else{
                final_transformationmatrix_vector_ << 0.5,0.5,0.5;
            }
            initializing = false;
        }


        // DIFFERENCE CALCULATION OF DELTA_A
        //delta_A.setZero();
        delta_A = final_transformationmatrix_vector_ - current_transformationmatrix_vector_;


        //output of Iteration step
        ROS_INFO("counter: %i", counter_);


        //abs_delta_A.setZero();
        // Calculate the absolute of the delta_A Vector
        for (int i=0;i<size_;i++){
            abs_delta_A(i) = abs(delta_A(i));
        }

        ROS_INFO("X-COOR: %f", current_transformationmatrix_vector_(9));
        ROS_INFO("Y-COOR: %f", current_transformationmatrix_vector_(10));
        ROS_INFO("Z-COOR: %f", current_transformationmatrix_vector_(11));


        // check if largest entry of absolute is smaller than error
        if (abs_delta_A.maxCoeff() < 0.05) {// uncomment this if limit of iterations is desired || counter_ == limit_) {
            for(int i=0; i<current_transformationmatrix_vector_.rows();i++){
                ROS_INFO("A_Matrix %f",current_transformationmatrix_vector_(i));
            }
            if(counter_ == limit_) {
               ROS_ERROR("Limit ueberstritten: %i", counter_);
            } else {
                ROS_INFO("FINISHED");
            }
            return transformVectorToArray(joint_angles_);

        } else {

            // calculate Jacobian and pseudoinverse
            J = calculateJacobian(joint_angles_, size_);
            pinvJ = J.completeOrthogonalDecomposition().pseudoInverse();

            // alternative functions
            //pinvJ = pinv_eigen_based(J); // function is commented below
            //pinvJ =  J.transpose() * (J*J.transpose()).inverse() ;
            //cvInvert(&J, &pinvJ, 0);
            //J = franka::Model::bodyJacobian(franka::Frame, joint_angles_);

            //correction of joint angles
            deltaQ = pinvJ * delta_A;
            joint_angles_ = joint_angles_ + deltaQ * incrementalStepSize;

            return incrementalStep();
        }
    }

    bool ik_jointAngles(inverse_kinematics::unserService::Request &req,
                        inverse_kinematics::unserService::Response &res) {

        // request joint angles
        for(int i=0;i<7;i++) {
            joint_angles_(i) = req.initial_joint_angles[i];
        }

        //TODO goal pos in service
        //PERFORM INVERSE KINEMATICS
        desired_joint_angles_ = incrementalStep();

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


    // Parse parameters specified in the .launch file
    std::string topic_name;
    int queue_size;
    node_handle.getParam("topic_name", topic_name);
    node_handle.getParam("queue_size", queue_size);

    // ------------- FOR SERVICE CONNECTION ------------------------------
    //auto client = node_handle.serviceClient<forward_kin::get_endeffector>(
    //"/forward_kin_node/get_endeffector"); $$$$$$$$$$$$$$$
    //IncrementalInverseKinematics inverse_kinematics(&client, size);
    // -------------------------------------------------------------------

    // SIZE OF JACOBIAN, RIGHT NOW ONLY 12 WORKS
    int size = 12;
    IncrementalInverseKinematics inverse_kinematics(size);
    ros::ServiceServer service = node_handle.advertiseService("unserService",&IncrementalInverseKinematics::ik_jointAngles,&inverse_kinematics);
    std::cout << inverse_kinematics.text;

    ros::spin();
}

