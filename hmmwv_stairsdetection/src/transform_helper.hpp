#ifndef TRANSFORM_HELPERS_HPP
#define TRANSFORM_HELPERS_HPP

#include <string>

#include <geometry_msgs/Point.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include "plane.hpp"

/**
 * @file transform_helpers.hpp
 * @brief Helper functions to do transformation.
 * @author Maximilian Hess <mail@ne0h.de>
 */

typedef struct Stairs {
	std::vector<Plane> steps;
} Stairs;

class TransformHelper {

public:

	TransformHelper(std::string cameraSetting, std::string worldFrameSetting)
			: m_cameraSetting(cameraSetting), m_worldFrameSetting(worldFrameSetting) {}

	/**
	 * Default destructor.
	 */
	~TransformHelper() {}

	/**
	 * Returns the axis-aligned bounding box of the point cloud
	 * @param cloud the input point cloud
	 * @param plane the output Plane
	 */
	void getAABB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Plane *plane);

	/**
	 * Transforms the coordinates of Stairs into world coordinates.
	 * @param stairs the input stairs
	 * @param cameraSetting the ROS topic for the camera frame
	 * @param worldFrameSetting the ROS topic for the world frame
	 */
	void transformStairsToWorldCoordinates(struct Stairs *stairs);

	/**
	 * Transforms a Point into world coordinates.
	 * @param p the input Point
	 * @param cameraSetting the ROS topic for the camera frame
	 * @param worldFrameSetting the ROS topic for the world frame
	 */
	bool transformToWorldCoordinates(pcl::PointXYZ *point);

	bool transformToWorldCoordinates(geometry_msgs::Point *point);

	bool transformToWorldCoordinates(Plane *plane);

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

	bool transformToBaseLinkCoordinates(geometry_msgs::Point *point);

	void buildStepFromAABB(Plane *plane, std::vector<pcl::PointXYZ> *points);

private:
	std::string m_cameraSetting;
	std::string m_worldFrameSetting;

};

#endif