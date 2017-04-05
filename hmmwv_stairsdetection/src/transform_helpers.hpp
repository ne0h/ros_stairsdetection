#ifndef TRANSFORM_HELPERS_HPP
#define TRANSFORM_HELPERS_HPP

#include <string>

#include <geometry_msgs/Point.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include "plane.hpp"

typedef struct Stairs {
	std::vector<Plane> steps;
} Stairs;

void getAABB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Plane *plane);

void transformStairsToWorldCoordinates(struct Stairs *stairs, std::string cameraSetting, std::string worldFrameSetting);

bool transformToWorldCoordinates(pcl::PointXYZ *p, std::string cameraSetting, std::string worldFrameSetting);

bool transformToWorldCoordinates(geometry_msgs::Point *p, std::string cameraSetting, std::string worldFrameSetting);

/**
 * Transforms a point from PCL coordinate system to ROS coordinate system.
 *
 * Documentation:
 * ROS: http://wiki.ros.org/geometry/CoordinateFrameConventions
 */
void transformPCLPointToROSPoint(pcl::PointXYZ *input, geometry_msgs::Point *output);

/**
 * Transforms a point from ROS coordinate system to PCL coordinate system.
 *
 * Documentation:
 * ROS: http://wiki.ros.org/geometry/CoordinateFrameConventions
 */
void transformROSPointToPCLPoint(geometry_msgs::Point *input, pcl::PointXYZ *output);

bool transformToBaseLinkCoordinates(geometry_msgs::Point *p, std::string cameraSetting, std::string worldFrameSetting);

#endif