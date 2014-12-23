#include <ros/ros.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "mrpt_reactivenav2d");
	ros::NodeHandle n;

	ros::spin();

	return 0;
}

