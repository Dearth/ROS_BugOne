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
#include <cmath>
#include "Location.h"
#include <vector>
#include <algorithm>

//My robot and all that it does.
class BugRobot {
	public:
		BugRobot(ros::NodeHandle&, double, double);
		void run();
	protected:
		void poseCallBack(const nav_msgs::Odometry::ConstPtr&);
		void laserScanCallBack(const sensor_msgs::LaserScan::ConstPtr&);
		void moveBug();
	private:
		//gereral useful calculations
		double calcBearing(position_s&, position_s&);
		void findNearWall();
		bool isLeft(position_s&);
		bool isRight(position_s&);

		//takes in a bearing and gives the approximate index 
		int calcRangeIndex(double);	

		//calculate a bearing with respect to the robots frame.
		double calcRobotBearing(position_s&);
		//general movement
		void moveTowardDest();
		bool isBlockedZero();
		geometry_msgs::Twist followWall();

		//bug algorithms
		void navigateWalls();	

		ros::Publisher cmd_vel_pub_;
		ros::Subscriber base_truth_sub_;
		ros::Subscriber laser_scan_sub_;

		position_s current_pos_;
		position_s destination_;
		position_s closest_wall_;
		
		std::vector<double> range_scan_;
		
		double range_incr_;
		double range_min_;
		bool following_wall_;
		bool in_corner_;

};

//Figure out if we are blocked for bug Zero
bool BugRobot::isBlockedZero() {
	double robotDest;
	int index, wallIndex; 

	findNearWall();
	
	wallIndex = calcRangeIndex(closest_wall_.bearing);
	robotDest = calcRobotBearing(destination_);
	index = calcRangeIndex(robotDest);

	if( !following_wall_ ) {
	
		//check wall index
		if( (wallIndex < 0) || (wallIndex > range_scan_.size()) ) {
			following_wall_ = false;
			return false;
		}

		//enter range looking for walls
		if ( range_scan_[wallIndex] < 0.8) {
		
			//If we can't see destination follow wall
			if ( (index > 1081) || (index < 0) ) {
				following_wall_ = true;
				return true;
			}

			if( range_scan_[index] < 5.0 ) {
				following_wall_ = true;
				return true;
			}

		}
	}
	else {
		
		//If we can't see destination follow wall
		if ( (index > 1081) || (index < 0) ) {
			following_wall_ = true;
			return true;
		}

		//Check if we have los 
		if ( range_scan_[index] >= 2 ) {
	
			//check wall index
			if( (wallIndex < 0) || (wallIndex > range_scan_.size()) ) {
				following_wall_ = false;
				return false;
			}

			//check if we are at a corner on the same side as dest
			if ( range_scan_[wallIndex] < 1.3 && closest_wall_.bearing < 0 && robotDest < 0 ) {
				return true;
			}
			else if (range_scan_[wallIndex] < 1.3 && closest_wall_.bearing > 0 && robotDest > 0 ){
				return true;
			}
			else {
				following_wall_ = false;
				return false;
			}
		}
		else {
			return true;
		}
	}
}

void BugRobot::navigateWalls() {
	geometry_msgs::Twist msg;

	/*
		INDEX 0F -PI/2 = 180
		INDEX OF PI/2 = 899
		INDEX OF 0 = 540
		HARDCODED DUE TO SEGFAULT
	*/

	if ( !in_corner_ ) {
		//if CC left corner
		if (range_scan_[540] < 0.7 && range_scan_[180] < 1.3 ) {
			in_corner_ = true;		
			msg.angular.z = -0.2;
		}
		//if CC right corner
		else if (range_scan_[540] < 0.7 && range_scan_[899] < 1.3 ) {
			in_corner_ = true;
			msg.angular.z = 0.2;
		}
		else {
			msg = followWall();
		}
	}
	else {
		if ( range_scan_[540] < 4.0 ) {
			if ( range_scan_[180] < 1.3 ) {
				msg.angular.z = -0.2;
			}
			else {
				msg.angular.z = 0.2;
			}
		}
		else {
				in_corner_ = false;
				msg = followWall();
		}
	}
	cmd_vel_pub_.publish(msg);
}

//movement tp follow the wall
geometry_msgs::Twist BugRobot::followWall() {
	geometry_msgs::Twist msg;
	double min_wall_dist = 0.8;
	double max_wall_dist = 1.2;

	int wallIndex = calcRangeIndex(closest_wall_.bearing);
	
	
	//if wall is on left
	if ( closest_wall_.bearing > 0 ) {
		if ( !isLeft(closest_wall_)) {
			
			if (closest_wall_.bearing > M_PI_2) {
				msg.angular.z = 0.1;
			}
			else {
				msg.angular.z = -0.1;
			}
		}
		else {
			if ( range_scan_[wallIndex] < min_wall_dist ) {
                msg.angular.z = -0.1;
                msg.linear.x = 0.25;
            }
			else if ( range_scan_[wallIndex] > max_wall_dist ) {
				msg.angular.z = 0.1;
				msg.linear.x = 0.25;
			}
            else {
				msg.linear.x = 0.25;
			}
		}
	}
	//if wall is on right
	else if ( closest_wall_.bearing < 0 ) {
		if ( !isRight(closest_wall_)) {

            if (closest_wall_.bearing < -M_PI_2) {
                msg.angular.z = -0.1;
            }
            else {
                msg.angular.z = 0.1;
            }
        }
        else {
            if ( range_scan_[wallIndex] < min_wall_dist ) {
                msg.angular.z = 0.1;
                msg.linear.x = 0.25;
            }
            else if ( range_scan_[wallIndex] > max_wall_dist ) {
                msg.angular.z = -0.1;
                msg.linear.x = 0.25;
            }
            else {
                msg.linear.x = 0.25;
            }
        }
    }
	else { 
		msg.linear.x = 0.25;
	}

	return msg;	
}

//movement if we are not blocked
void BugRobot::moveTowardDest() {
	geometry_msgs::Twist msg;
	in_corner_ = false;

	destination_.bearing = calcBearing(current_pos_, destination_);
	
	if ( !destination_.isEqualAproxLoc(current_pos_) ) {
		if( !destination_.isEqualAproxBearing(current_pos_) ) {
			if ( destination_.bearing > current_pos_.bearing ) {
				msg.angular.z = 0.1;
			}
			else {
				msg.angular.z = -0.1;
			}
		}
		else {
			msg.linear.x = 0.25;
		}
	}
	else {
		msg.linear.x = msg.linear.y = msg.linear.z = msg.angular.z = 0;
	}
	
	cmd_vel_pub_.publish(msg);
}

//Initialize all values of the robot, set up subscriber and publisher.
BugRobot::BugRobot(ros::NodeHandle& nh, double goal_x, double goal_y) {

	// save my goal
	destination_.x = goal_x;
	destination_.y = goal_y;
	following_wall_ = false;
	in_corner_ = false;

	range_scan_.resize(1081);
	
	//Create publisher for cmd_vel
	cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);

	//Create subscription to base_scan
	laser_scan_sub_ = nh.subscribe("/base_scan", 1, &BugRobot::laserScanCallBack, this);

	//Create a subscription to base_pose_ground_truth
	base_truth_sub_ = nh.subscribe("/base_pose_ground_truth", 1, &BugRobot::poseCallBack, this);

}

//base_pose_ground_truth call back
void BugRobot::poseCallBack(const nav_msgs::Odometry::ConstPtr& msg) {
	tf::Pose pose;

	//Get x and y values
	current_pos_.x = msg->pose.pose.position.x;
	current_pos_.y = msg->pose.pose.position.y;
	
	//get bearing as angle
	tf::poseMsgToTF(msg->pose.pose, pose);
	current_pos_.bearing = tf::getYaw(pose.getRotation());

}

//base_scan call back
void BugRobot::laserScanCallBack(const sensor_msgs::LaserScan::ConstPtr& msg) {
	range_incr_ = msg->angle_increment;
	range_min_ = msg->angle_min;

	for( int i = 0; i < range_scan_.size(); ++i ) {
		range_scan_[i] = msg->ranges[i];
	}
}

//calculate nearest wall
void BugRobot::findNearWall() {
	position_s temp;
	int index = 0;
	double close_range = 31;

	for ( int i = 0; i < range_scan_.size(); ++i ) {
		if ( close_range > range_scan_[i] ) {
			close_range = range_scan_[i];
			index = i;
		}
	}

	closest_wall_.x = (close_range * sin((index*range_incr_)+range_min_)) + current_pos_.x;
	closest_wall_.y = (close_range * cos((index*range_incr_)+range_min_)) + current_pos_.y;

	closest_wall_.bearing = (index*range_incr_) + range_min_;

}

//Calculat the angle of a bearing with respect to the robots frame.
double BugRobot::calcRobotBearing(position_s& dest) {
	position_s roboDest;
	position_s origin(0,0,0);

	double x = dest.x - current_pos_.x;
	double y = dest.y - current_pos_.y;

	roboDest.x = x * cos(current_pos_.bearing) + y * sin(current_pos_.bearing);
	roboDest.y = -(x * sin(current_pos_.bearing)) + y * cos(current_pos_.bearing);
	roboDest.bearing = calcBearing(origin, roboDest);

	return roboDest.bearing;
	
}

//Calculate the approximate index of a value in range_scan_ given a bearing
int BugRobot::calcRangeIndex(double bearing) {
	return (bearing - range_min_) / range_incr_;

}

//Calculate the bearing between two points.
double BugRobot::calcBearing(position_s& current, position_s& dest) {
	return atan2(dest.y - current.y, dest.x - current.x);
}

//Calculate if a given position is on our left or on our right.
bool BugRobot::isLeft(position_s& dest) {
	position_s left(0,0,M_PI_2);

	return dest.isEqualAproxBearing(left);

}

bool BugRobot::isRight(position_s& dest) {
	position_s right(0,0,-M_PI_2);

	return dest.isEqualAproxBearing(right);
}

//Move the robot a single step
void BugRobot::moveBug() {
	
	if ( isBlockedZero() ) {
		navigateWalls();
	}
	else {
		moveTowardDest();
	}
}

void BugRobot::run() {
	ros::Rate rate(30);

	while(ros::ok()){

		moveBug();

		ros::spinOnce();
		rate.sleep();
	}
}

#endif





























