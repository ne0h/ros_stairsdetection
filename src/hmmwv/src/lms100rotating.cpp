#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using namespace ros;
using namespace std;

void velocityCallback(const geometry_msgs::Twist& msg) {
}

int main(int argc, char **argv) {
	// init ros
	init(argc, argv, "lms100rotating"); // rotating LMS100
	NodeHandle n;
	Subscriber sub = n.subscribe("turtle1/cmd_vel", 1, velocityCallback);
	ROS_INFO("lms100rotating up and running.");
	
	// enter ros loop and wait for callbacks
	spin();
	return 0;
}
