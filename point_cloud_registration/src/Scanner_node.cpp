#include <eigen_conversions/eigen_msg.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "point_cloud_registration/PCJScombined.h"

class Scanner {
 public:
  Scanner(ros::NodeHandle& nh)
      : nh_(nh),
        nextPose(0),
        pub_(nh_.advertise<point_cloud_registration::PCJScombined>("PCJScombined", 1)),
        pose_publisher_(nh_.advertise<std_msgs::Float64MultiArray>("/move_command",1))
  {
    for (int i = 0; i < 15; ++i) {
      goal_poses[i].resize(7);
    }
    goal_poses[0] << 0.07975098222703264, 0.1912801323522601, 0.016800910318580282, -1.780517308656089, -0.43001809130774604, 2.840706754071704, 1.2763794419674155;
    goal_poses[1] << 0.07423677020502224, -0.16895207870170834, -0.01739294999455757, -2.148493787373226, -0.4594188762764029, 2.9986082050005596, 1.2267582378234292;
    goal_poses[2] << 0.09911098749072929, -0.1239977193691705, -0.00531071934126161, -2.048764962082538, -0.4053728662024609, 2.8943427835385838, 1.2215459212439252;
    goal_poses[3] << -0.10126473527490241, -0.9280972911700696, 0.2613660008405384, -2.6643017023718905, -0.24911827067092612, 3.21774939769697, 1.260410547982497;
    goal_poses[4] << 0.021348165159685576, -1.3986401582171828, 0.2485869409865171, -2.8720675798107402, -0.2596319724188911, 3.058391145547231, 1.2582345738063254;
    goal_poses[5] << 0.07564599031697936, -1.7455189008841003, 0.20152863198662244, -2.997883944977146, -0.3529262597649186, 2.978800730860714, 1.3184717867661728;
    goal_poses[6] << 0.05693500763320076, -1.4830985697520966, 0.6208186084178455, -2.801915834330165, -0.22718658990903615, 2.9779661087989724, 1.3213901261949699;
    goal_poses[7] << -0.3040832270592223, -1.1170643709856485, 0.8441226244642022, -2.5022547237563177, -0.20441106857447605, 3.013031459465425, 1.3456559313652996;
    goal_poses[8] << -0.6924211229662991, -0.8604797595174742, 1.0764434815526132, -2.2717824239898143, -0.18019897478719868, 3.014822743472443, 1.3447165227648283;
    goal_poses[9] << -1.078069790668655, -0.8071784893457317, 1.449906561240025, -1.8885478407896785, -0.12678724956993626, 2.786309055169423, 1.2694327265799883;
    goal_poses[10] << 0.9208467507134637, -0.7069723573017217, -1.058362939081694, -2.0975300465988833, -0.2728774133437191, 2.7571864351872293, 0.8424597964588909;
    goal_poses[11] << 0.6311146891707178, -0.9951198768922268, -0.8815044550454316, -2.2933741094354994, -0.33896638763599984, 2.8065129842521146, 0.8411473497466908;
    goal_poses[12] << 0.38011963067974963, -1.3720267832039361, -0.7637782996345541, -2.427091076814473, -0.37758252994543134, 2.7370390356381735, 0.8413532063090357;
    goal_poses[13] << 0.9900714515761325, -1.7453625325281177, 1.437348222528641, -2.760951613952724, -0.9397039766276116, 2.060468782265981, 2.276330765972267;
    goal_poses[14] << 0.19544769447960644, 0.5514320451719299, 0.09095485297256316, -1.368262226205125, -0.5614982806843187, 2.6206327856381733, 1.2989454467564898;
  }

 private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Publisher pose_publisher_;
  ros::Subscriber sub_;
  std::array<Eigen::VectorXd, 15> goal_poses;
  //message_filters::Subscriber<sensor_msgs::PointCloud2> filtered_cloud;
  //message_filters::Subscriber<sensor_msgs::JointState> jointStates;
  //bool published_message;

  //const int pc_queue_size = 50;
  int nextPose;
  const int js_queue_size = 500;
  const int rounding_precision = 10000;

  //function from which we will send our goal pose to the path planning node
  void sendGoalPose(){
    if (nextPose == 0){
      ros::Duration(1).sleep();
    }
    std_msgs::Float64MultiArray outputPose;
    tf::matrixEigenToMsg(goal_poses[nextPose], outputPose);
    pose_publisher_.publish(outputPose);
  }

  //subscribe to to the joint_states topic
  void getPose(){
    std::cerr << "started get pose" << std::endl;
    sub_ = nh_.subscribe<sensor_msgs::JointState>("/franka_state_controller/joint_states_desired", js_queue_size, boost::bind(&Scanner::comparePose, this, _1));
    ros::spin();
  }

  //check if the joint states reached the desired position. if so, increment the nextPose counter and enter the getPointCloud function
  void comparePose(const sensor_msgs::JointStateConstPtr& joint_state){
    sensor_msgs::JointState tempJS;
    sensor_msgs::JointState rounded_goal_pose;

    for (int i = 0; i < 7; ++i) {
      tempJS.position.push_back(round(joint_state->position[i]*rounding_precision)/rounding_precision);
      rounded_goal_pose.position.push_back(round(goal_poses[nextPose][i]*rounding_precision)/rounding_precision);
    }

    if (rounded_goal_pose.position == tempJS.position){
      std::cerr << "goal pose reached!" << std::endl;
      //ros::Duration(0.2).sleep();
      nextPose++;
      getPointCloud();
    }
  }

  //Subscribe to the pointcloud and jointstates topic and match them by time stamp
  //boolean "published_message", to check (in the callback function) if there was already an message published for this goal pose
  void getPointCloud(){

/*    std:cerr << "entered getPC" << std::endl;
    published_message = false;
    filtered_cloud.subscribe(nh_, "/points2", pc_queue_size);
    jointStates.subscribe(nh_, "/joint_states", js_queue_size);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::JointState> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(js_queue_size), filtered_cloud, jointStates);
    sync.registerCallback(boost::bind(&Scanner::publishMessage, this, _1, _2));
    ros::spin();*/


    //when we work with static poses, there is no need for time stamp matching, so we could just take the next incoming pointcloud and jointstates and publish them as a combined message
    //this would also make the callback function (publishMessage) unnecessary
    //ros::Duration(0.5).sleep();
    sensor_msgs::PointCloud2ConstPtr PC = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/k4a/points2");
    sensor_msgs::JointStateConstPtr JS = ros::topic::waitForMessage<sensor_msgs::JointState>("/franka_state_controller/joint_states_desired");

     //make sure messages have been received (apparently not necessary, but maybe we should still keep it)
     //if(JS != NULL && PC != NULL){
       point_cloud_registration::PCJScombined PCJS;
       PCJS.PC = *PC;
       PCJS.JS = *JS;
       pub_.publish(PCJS);
       if (nextPose<sizeof(goal_poses)) {
         sendGoalPose();
       }
     //}
  }

  //callback function to publish the matched pc and js
  //first check, if there was already a message published for this pose. if so, do nothing
  //otherwise, unsubscribe from the topics, so no more messages come in and publish the received pc+js
  //as a last step, send the next goalPose
/*  void publishMessage(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const sensor_msgs::JointStateConstPtr& joint_states){
    std:cerr << "entered publish message" << std::endl;
      if(published_message){
        return;
      }
    filtered_cloud.unsubscribe();
    jointStates.unsubscribe();
    point_cloud_registration::PCJScombined PCJS;
    PCJS.PC = *cloud_msg;
    PCJS.JS = *joint_states;
    pub_.publish(PCJS);
    published_message = true;
    if (nextPose<sizeof(goal_poses)){
      sendGoalPose();
    }else{
      nh_.shutdown();
      ros::shutdown();
      //this does not shut down the whole node for some reason
      //TODO: find a better solution
    }
  }*/

  //public function to start the scanning, first send a goal pose and then start checking, if it has been reached yet
 public:
  void startScan(){
      sendGoalPose();
      getPose();
  }
};

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "Scanner_node");
  ros::NodeHandle nh("~");

  Scanner scan(nh);
  scan.startScan();
}

