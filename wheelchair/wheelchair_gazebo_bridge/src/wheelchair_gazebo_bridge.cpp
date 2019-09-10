#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <iostream>

#include <string.h>
#include <functional>
#include <boost/functional.hpp>

extern int clock_gettime(clockid_t clk_id, struct timespec *res);


using namespace std;

template<typename T, typename F>
ros::Subscriber create_bridge(ros::NodeHandle& node, const std::string& name, int max_count, F function){
	return node.subscribe<T>(name.c_str(), max_count, static_cast<boost::function<void(const T&)>>(std::bind(function, std::placeholders::_1)));
}


int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "wheelchair_gazebo_bridge");

 /* ==========================================================
     publisherとsubscriberの初期化:
   ========================================================== */
  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/wheelchair/odom", 1000);
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/wheelchair/diff_drive_controller/cmd_vel", 1000);
  ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose>("/wheelchair/pose", 1000);
  ros::Publisher obs_pose_pub = n.advertise<geometry_msgs::Pose>("obs_pose", 1000);

  ros::Subscriber odom_sub = create_bridge<nav_msgs::Odometry>(n, "/wheelchair/diff_drive_controller/odom", 1000, [&](const auto& msg){odom_pub.publish(msg);});
  ros::Subscriber cmd_vel_sub = create_bridge<geometry_msgs::Twist>(n, "/wheelchair/cmd_vel", 1000, [&](const auto& msg){cmd_vel_pub.publish(msg);});
  ros::Subscriber pose_sub = create_bridge<gazebo_msgs::ModelStates>(n, "/gazebo/model_states", 1000, [&](const auto& msg){
  																															for (int i = 0; i < msg.name.size(); ++i){
  																																	if(msg.name[i] == "wheelchair"){
  																																		pose_pub.publish(msg.pose[i]);
                                                                      printf("OK\n");
  																																	}
																																	else if (msg.name[i] == "obstacle"){
																																		obs_pose_pub.publish(msg.pose[i]);
																																	}
  																																}
  																															});

  ros::spin();
  return 0;
}
