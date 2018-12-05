#include <ros/ros.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <robot_localization/SetPose.h>
// #include <setPose.h>

nav_msgs::Odometry true_pose;
nav_msgs::Odometry noisy_pose;

std_msgs::Bool submerged_status;
bool start = true; 
bool updated_pos = false;
ros::ServiceClient client;

ros::Publisher noisePosPub;

void true_position_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  true_pose=*msg;

  if (submerged_status.data==1) // The robot is underwater
  {
    ROS_INFO_STREAM("submerged_status "<<submerged_status<<"\n");

    noisy_pose.header = true_pose.header;
    noisy_pose.child_frame_id = "rexrov/base_link_imu"; 
    noisy_pose.pose.pose.position.x = true_pose.pose.pose.position.x + (-1+2*(double)std::rand()/(RAND_MAX))*0.1;
    noisy_pose.pose.pose.position.y = true_pose.pose.pose.position.y + (-1+2*(double)std::rand()/(RAND_MAX))*0.1;
    noisy_pose.pose.pose.position.z = true_pose.pose.pose.position.z + (-1+2*(double)std::rand()/(RAND_MAX))*0.1;
    noisy_pose.pose.pose.orientation.x = 0.0;
    noisy_pose.pose.pose.orientation.y = 0.0;
    noisy_pose.pose.pose.orientation.z = 0.0;
    noisy_pose.pose.pose.orientation.w = 0.0;

    noisy_pose.twist = true_pose.twist;


  }

  else {
    noisy_pose = true_pose;
    noisy_pose.child_frame_id = "rexrov/base_link_imu";
  }

  noisePosPub.publish(noisy_pose);

}

void submerged_status_callback(const std_msgs::Bool::ConstPtr& msg)
{
  submerged_status=*msg;


}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "init_state_publisher");
	ros::NodeHandle n;

	ros::Subscriber truePosSub = n.subscribe("rexrov/pose_gt", 10, &true_position_callback);
	ros::Subscriber submergedStatusSub = n.subscribe("rexrov/is_submerged", 10, &submerged_status_callback);

  noisePosPub = n.advertise<nav_msgs::Odometry>("/uuv_imu_pose",1000);    


	ros::Rate loop_rate(50);

  	while(ros::ok()) 
  	{
  		start = false;
    	ros::spinOnce();
    	loop_rate.sleep();
    	// ROS_INFO_STREAM("submerged_status "<<submerged_status<<"\n");

    }

    return 0; 
}