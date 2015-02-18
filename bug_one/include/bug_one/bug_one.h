#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <nav_msgs/Odometry.h>

//a small struct to hold my position.
typedef struct {
	double x;
	double y;
	double bearing;
} position;

//My robot and all that it does.
class BugRobot {
	public:
		bug_robot(ros::NodeHandle&, double, double);
		void run();
	private:
		double x_dest_, y_dest_;
		ros::Publisher cmd_vel_pub_;
		ros::Subscriber base_truth_sub_;
		position current_pos_
};

//Initialize all values of the robot, set up subscriber and publisher.
BugRobot::BugRobot(ros::NodeHandle& nh, double goal_x, double goal_y) {
	
	// save my goal
	x_dest_ = goal_x;
	y_dest_ = goal_y;

	//Create publisher for cmd_vel
	cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);

	//Create subscription to base_pose_ground_truth
	
