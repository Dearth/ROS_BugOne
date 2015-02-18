#include "bugrobot.h"
/**
 *  
 * This tutorial demonstrates simple sending of messages over the ROS system
 * and controlling a robot.
 *   
 */


int main(int argc, char **argv)
{
	/// Name your node
	ros::init(argc, argv, "bug_one");
	/// Every ros node needs a node handle, similar to your usual  file handle.
	ros::NodeHandle nh_;
	/// Publisher object that decides what kind of topic to publish and how fast.
	double dest_x, dest_y;
	
	BugRobot bug(nh_, 1, 2);
	bug.run();
	return 0;
}

