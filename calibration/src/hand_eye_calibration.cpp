//
// Created by nico on 11.06.21.
//

#include <iostream>
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"

using namespace std;
using namespace cv;

// containers for calibration data
vector<TransformStamped> allRobotPoses;
Size imgSize;
vector<cv::Mat> rvecs, tvecs;
vector<Mat> R_gripper2base, t_gripper2base;

vector<vector<Point2f>> allCharucoCorners;
vector<vector<int>> allCharucoIds;


void read_camcal_data(){
    // read text files
    std::ifstream robot_poses("joint_states.txt");

    int rows = 0;
    std::string line;
    while (std::getline(robot_poses, line)) {

        std::istringstream stream(line);

        char sep; //comma!
        double x;
        // read *both* a number and a comma:
        while (stream >> x && stream >> sep) {

            depthImg.push_back(x);
        }
        rows ++;
    }
    // push to

// reshape to 2d:
    depthImg = depthImg.reshape(1,rows);

}

void calculate_pose(){
    forward_kin::get_endeffector srv;
    sensor_msgs::JointState joint_state_msg;
    joint_state_msg  = *(ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states",ros::Duration(10)));
    ros::ServiceClient client = n.serviceClient<forward_kin::get_endeffector>("forward_kin_node/get_endeffector");
    boost::array<double, 7> arr = {joint_state_msg.position[0], joint_state_msg.position[1],joint_state_msg.position[2],
                                   joint_state_msg.position[3], joint_state_msg.position[4],joint_state_msg.position[5],
                                   joint_state_msg.position[6]};
    srv.request.joint_angles = arr;
    auto a = client.call(srv);
    if (a)
    {
        ROS_INFO("Endpos: %f", srv.response.end_effector_pos[0]);
    }
    else
    {
        ROS_ERROR("Failed to call service forward_kin");
        return 1;
    }
    R_gripper2base, t_gripper2base;
    allRobotPoses.pushback({(float)srv.response.end_effector_pos[0], (float)srv.response.end_effector_pos[1], (float)srv.response.end_effector_pos[2]});
    for(int i=0; i<allRobotPoses.size(); i++){
        Mat R, t;
        transform2rv(allRobotPoses[i], R, t);
        R_gripper2base.push_back(R);
        t_gripper2base.push_back(t);
    }
}

void hand_eye(){
    ROS_INFO("Starting hand-eye calibration.");

    // collecting transformations for all poses

    for(int i=0; i<allRobotPoses.size(); i++){
        Mat R, t;
        transform2rv(allRobotPoses[i], R, t);
        R_gripper2base.push_back(R);
        t_gripper2base.push_back(t);
    }

    // performing hand-eye calibration
    Mat R_cam2gripper, t_cam2gripper;
    calibrateHandEye(R_gripper2base, t_gripper2base, rvecs, tvecs, R_cam2gripper, t_cam2gripper);
    cout << "hand eye transformation: translation:\n" << t_cam2gripper << endl;
    cout << "hand eye transformation: rotation:\n" << R_cam2gripper << endl;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "hand_eye_calibration");
    Calibrator obj;
    ros::Rate loop_rate(50);
    cout << "press 's' to add current keyframe, 'c' to calibrate, 'q' to quit program" << endl;
    while (ros::ok()){
        int key = obj.showImage();
        if(key == 's')
        {
            obj.saveData();
        }
        if(key == 'c')
        {
            obj.calibrate();
        }
        if(key == 'q')
        {
            break;
        }
        ros::spinOnce();
    }
    return 0;
}