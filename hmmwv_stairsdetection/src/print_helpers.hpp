#ifndef PRINT_HELPERS_HPP
#define PRINT_HELPERS_HPP

#include <geometry_msgs/Point.h>

#include "plane.hpp"

void printROSPoint(geometry_msgs::Point *p);

void printPlane(Plane *plane);

void printStairs(struct Stairs *stairs);

void printStairs(std::vector<struct Stairs> *stairs);

void printPoint(pcl::PointXYZ *p);

#endif