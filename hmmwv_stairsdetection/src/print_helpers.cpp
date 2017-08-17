#include "print_helpers.hpp"
#include "transform_helper.hpp"

#include <vector>

void printROSPoint(geometry_msgs::Point *p) {
	ROS_INFO("Point: %f %f %f", p->x, p->y, p->z);
}

void printPlane(Plane *plane) {
	ROS_INFO("AABB: %f %f %f -> %f %f %f", 	plane->getMin().x, plane->getMin().y, plane->getMin().z,
											plane->getMax().x, plane->getMax().y, plane->getMax().z);
}

void printStairs(struct Stairs *stairs) {
	for (std::vector<Plane>::iterator it = stairs->steps.begin(); it != stairs->steps.end(); it++) {
		ROS_INFO("Min: %f | %f | %f", (*it).getMin().x, (*it).getMin().y, (*it).getMin().z);
		ROS_INFO("Max: %f | %f | %f", (*it).getMax().x, (*it).getMax().y, (*it).getMax().z);
		ROS_INFO("----------------");
	}
}

void printStairs(std::vector<struct Stairs> *stairs) {
	for (std::vector<struct Stairs>::iterator it = stairs->begin(); it != stairs->end(); it++) {
		ROS_INFO("################################");
		printStairs(&(*it));
	}
	ROS_INFO("################################");
}

void printPoint(pcl::PointXYZ *p) {
	ROS_INFO("Point: %f %f %f", p->x, p->y, p->z);
}