#include "print_helpers.hpp"

#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

void print(Step &step) {
    ROS_INFO("%s", step.toString().c_str());
}

void print(std::vector<Step> &steps) {
    for (std::vector<Step>::iterator it = steps.begin(); it != steps.end(); it++) {
		print(*it);
	}
}

void print(Stairway &stairway) {
   print(stairway.getSteps());
}

void print(std::vector<Stairway> &stairways) {
    for (std::vector<Stairway>::iterator it = stairways.begin(); it != stairways.end(); it++) {
		print(*it);
	}
}

void print(geometry_msgs::Point &p) {
    ROS_INFO("Point: %f %f %f", p.x, p.y, p.z);
}