#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <geometry_msgs/Odometry.h>
#include <bug_one/bug_one.h>

/**
 *  
 * This tutorial demonstrates simple sending of messages over the ROS system
 * and controlling a robot.
 *   
 */


void locationCallBack(const nav_msgs::Odometry::ConstPTr& msg){
	
}

int main(int argc, char **argv)
{
	/// Name your node
	ros::init(argc, argv, "bug_one");
	/// Every ros node needs a node handle, similar to your usual  file handle.
	ros::NodeHandle nh_;
	/// Publisher object that decides what kind of topic to publish and how fast.
	ros::Publisher cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	// We will be sending commands of type "twist"
	geometry_msgs::Twist base_cmd;
	// Listen to base_pose_groud_truth
	ros::Subscriber base_pose_sub = nh_.subscribe("/base_pose_ground_truth",1,locationCallBack);
	// User input
	double dest_x, dest_y;

	std::cout << "Please, enter destination x coord:";
	std::cin >> dest_x;

	std::cout << "Enter destination y coord:";
	std::cin >> dest_y;

	/// The main loop will run at a rate of 10Hz, i.e., 10 times per second.
	ros::Rate loop_rate(10);
	/// Standard way to run ros code. Will quite if ROS is not OK, that is, the master is dead.
	while (ros::ok())
	{
		if 
			base_cmd.angular.z = 0.75;
			base_cmd.linear.x = 0.25;
		/// Here's where we publish the actual message.
		cmd_vel_pub_.publish(base_cmd);
		/// Spin the ros main loop once 
		ros::spinOnce();
		/// Sleep for as long as needed to achieve the loop rate.
		loop_rate.sleep();
	}
	return 0;
}

