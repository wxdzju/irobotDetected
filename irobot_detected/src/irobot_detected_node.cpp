#include <ros/ros.h>
#include "colorDetected.h"
using namespace std;
int main(int argc, char** argv)
{
	ros::init(argc, argv, "irobot_detected_node");
	ros::NodeHandle nh;
	irobotDetected::ColorDetected irobot(nh);
	ros::spin();
	return 0;
}