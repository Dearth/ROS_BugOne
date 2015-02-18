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
	
	BugRobot bug(nh_, 0, 0);
	bug.run();
	return 0;
}

