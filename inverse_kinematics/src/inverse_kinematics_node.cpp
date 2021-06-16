/*
 * This is the example of Tutorial 5 to test and understand incremental inverse kinematics
 * author: Tolga Kartal, Sean Maroofi
 * added: 29.5.21
 * last edited: 11.6.21
 */







#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
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
        incrementalStepSize = 0.01;
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
        A_total=a_.at(0)*a_.at(1)*a_.at(2)*a_.at(3)*a_.at(4)*a_.at(5)*a_.at(6)*a_.at(7);

        vector<< A_total(0,0), A_total(1,0), A_total(2,0),
                A_total(0,1), A_total(1,1), A_total(2,1),
                A_total(0,2), A_total(1,2), A_total(2,2),
                A_total(0,3), A_total(1,3), A_total(2,3);

        return vector;
    }

    //END OF FORWARD


    /*
* This function can be used to calculate the 4x4 Transformation Matrix vor a given 6x1 endeffector. The function uses Euler Angles with the X,Y,Z convention
*
* @param: VectorXd endeffector
* returns: Matrix4d Transformationmatrix
*/
Matrix4d convertEndeffectorToTransformation(VectorXd endeffector){
        Matrix3d rotation;
        Matrix4d matrix;

        rotation = AngleAxisd(endeffector(3), Vector3d::UnitX())
                    * AngleAxisd(endeffector(4), Vector3d::UnitY())
                    * AngleAxisd (endeffector(5), Vector3d::UnitZ());

        matrix << rotation(0,0), rotation(0,1), rotation(0,2), endeffector(0),
                  rotation(1,0), rotation(1,1), rotation(1,2), endeffector(1),
                  rotation(2,0), rotation(2,1), rotation(2,2), endeffector(2),
                                       0,                      0,                     0,                    1;

        return matrix;
    }


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
                VectorXd endeffectorPos(6);
                ROS_INFO("Ich war zuerst hier");
                endeffectorPos << 0.259209, 0.403714, 0.617182, 1.121, 0.982, 0.31457;
              Matrix4d final_transformationmatrix;
                ROS_INFO("Ich war dann hier");
              final_transformationmatrix << convertEndeffectorToTransformation(endeffectorPos);
                ROS_INFO("Ich war zum Schluss hier");
              //0.377556, -0.029182, 0.925526, 0.249209, 0.92475, 0.063488, -0.375232, 0.403714, -0.047806, 0.997552, 0.05096 ,0.617182, 0, 0, 0, 1;



            final_transformationmatrix_vector_ = convert4DMatrixTo12DVector(final_transformationmatrix);
                for (int i=0; i<final_transformationmatrix_vector_.rows();i++){
                    ROS_INFO("Final Matrix %f", final_transformationmatrix_vector_(i));
                }

                /*  0.244948, 0.96417, 0.101854,
                       0.076194, -0.123872, 0.989368,
                       0.966538, -0.234584, -0.103804,
                       0.250202, 0.444379, 0.632892;  */    // ONLY FOR TESTING

            } else{
                final_transformationmatrix_vector_ << 0.5,0.5,0.5;
            }
            initializing = false;
        }


        // DIFFERENCE CALCULATION OF DELTA_A
        delta_A = final_transformationmatrix_vector_ - current_transformationmatrix_vector_;


        //output of Iteration step
        ROS_INFO("counter: %i", counter_);

        // Calculate the absolute of the delta_A Vector
        for (int i=0;i<size_;i++){
            abs_delta_A(i) = abs(delta_A(i));
        }

        ROS_INFO("Max delta %f",abs_delta_A.maxCoeff());

        // check if largest entry of absolute is smaller than error
        if (abs_delta_A.maxCoeff() < 0.01) {// uncomment this if limit of iterations is desired || counter_ == limit_) {

            if(counter_ == limit_) {
               ROS_ERROR("Limit ueberstritten: %i", counter_);
            } else {
                ROS_INFO("FINISHED");
            }
            return transformVectorToArray(joint_angles_);

        } else {

            // calculate Jacobian and pseudoinverse
            J = calculateJacobian(joint_angles_, size_);

            //correction of joint angles
            deltaQ = J.completeOrthogonalDecomposition().solve(delta_A);
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

        VectorXd finalA2 = get_transformation_Vector();
        for(int i=0; i<7;i++){
            ROS_INFO("desired jointAngles %f",desired_joint_angles_[i]);
        }


        res.ik_jointAngles = {desired_joint_angles_[0], desired_joint_angles_[1],
                              desired_joint_angles_[2], desired_joint_angles_[3],
                              desired_joint_angles_[4], desired_joint_angles_[5], desired_joint_angles_[6]};
        return true;
    }
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

