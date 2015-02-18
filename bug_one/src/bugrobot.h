#ifndef _BUGROBOT_H_
#define _BUGROBOT_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>

//a small struct to hold my position.
typedef struct Location {
	double x;
	double y;
	double bearing;
	
	Location() : x(0), y(0), bearing(0) {};

} position_s;

//My robot and all that it does.
class BugRobot {
	public:
		BugRobot(ros::NodeHandle&, double, double);
		void run();
	protected:
		void poseCallBack(const nav_msgs::Odometry::ConstPtr&);
		void laserScanCallBack(const sensor_msgs::LaserScan::ConstPtr&);

	private:
		double x_dest_, y_dest_;
		ros::Publisher cmd_vel_pub_;
		ros::Subscriber base_truth_sub_;
		ros::Subscriber laser_scan_sub_;
		position_s current_pos_;
};

//Initialize all values of the robot, set up subscriber and publisher.
BugRobot::BugRobot(ros::NodeHandle& nh, double goal_x, double goal_y) {

	// save my goal
	x_dest_ = goal_x;
	y_dest_ = goal_y;
	
	//Create publisher for cmd_vel
	cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);

	//Create subscription to base_scan
	laser_scan_sub_ = nh.subscribe("/base_scan", 1, &BugRobot::laserScanCallBack, this);

	//Create a subscription to base_pose_ground_truth
	base_truth_sub_ = nh.subscribe("/base_pose_ground_truth", 1, &BugRobot::poseCallBack, this);

}

void BugRobot::poseCallBack(const nav_msgs::Odometry::ConstPtr& msg) {
	tf::Pose pose;

	//Get x and y values
	current_pos_.x = msg->pose.pose.position.x;
	current_pos_.y = msg->pose.pose.position.y;
	
	//get bearing as angle
	tf::poseMsgToTF(msg->pose.pose, pose);
	current_pos_.bearing = tf::getYaw(pose.getRotation());

}

void BugRobot::laserScanCallBack(const sensor_msgs::LaserScan::ConstPtr& msg) {}

void BugRobot::run() {
	ros::Rate rate(30);

	while(ros::ok()){

		std::cerr << current_pos_.x << " ";
		std::cerr << current_pos_.y << " ";
		std::cerr << current_pos_.bearing << std::endl;
		
		ros::spinOnce();
		rate.sleep();
	}
}

#endif
