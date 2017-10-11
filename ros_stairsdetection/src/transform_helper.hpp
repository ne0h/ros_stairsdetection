#pragma once

#include <string>
#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include "plane.hpp"
#include "stairway.hpp"

/**
 * @file transform_helpers.hpp
 * @brief Helper functions to do transformation.
 * @author Maximilian Hess <mail@ne0h.de>
 */

class TransformHelper {

public:

	/**
	 * Default constructor.
	 */
	TransformHelper() {}

	/**
	 * Creates a new TransformHelper.
	 * @param cameraSetting the ROS topic for the camera frame
	 * @param worldFrameSetting the ROS topic for the world frame
	 */
	TransformHelper(std::string cameraFrameSetting, std::string robotFrameSetting, std::string worldFrameSetting,
		tf2_ros::Buffer *tfBuffer)
			: m_cameraFrameSetting(cameraFrameSetting), m_robotFrameSetting(robotFrameSetting),
				m_worldFrameSetting(worldFrameSetting) {
		
		m_tfBuffer = tfBuffer;
	}

	/**
	 * Default destructor.
	 */
	~TransformHelper() {}

	/**
	 * Returns the axis-aligned bounding box of a point cloud
	 * @param cloud the input point cloud
	 * @param plane the output Plane
	 */
	void getAABB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Plane &plane);

	/**
	 * Transforms the coordinates of Stairs into world coordinates.
	 * @param stairs the input stairs
	 */
	void transformToWorldCoordinates(Stairway &stairway) {
		for (std::vector<Plane>::iterator it = stairway.getSteps().begin(); it != stairway.getSteps().end(); it++) {
			transformToWorldCoordinates(*it);
		}
	}

	/**
	 * Transforms the coordinates of a ROS point to world coordinates.
	 * @param point the input point
	 */
	bool transformToWorldCoordinates(geometry_msgs::Point &point) {
		return transform(point, m_worldFrameSetting, m_robotFrameSetting);
	}

	/**
	 * Transforms the coordinates of a Plane to world coordinates
	 * @param plane the input Plane
	 */
	bool transformToWorldCoordinates(Plane &plane);

	bool transformToRobotCoordinates(geometry_msgs::Point &point) {
		return transform(point, m_robotFrameSetting, m_cameraFrameSetting);
	}

	bool transformToRobotCoordinates(Plane &plane) {
		geometry_msgs::Point newMin = plane.getMin(), newMax = plane.getMax();
		if ((!transformToRobotCoordinates(newMin)) || (!transformToRobotCoordinates(newMax))) {
			return false;
		}

		plane.setMinMax(newMin, newMax);
		return true;
	}

	/**
	 * Transforms a point from PCL coordinate system to ROS coordinate system.
	 *
	 * Documentation:
	 * ROS: http://wiki.ros.org/geometry/CoordinateFrameConventions
	 */
	void transformPCLPointToROSPoint(pcl::PointXYZ &input, geometry_msgs::Point &output);

	/**
	 * Transforms the coorindates of a ROS point to world coordinates.
	 * @param point the ROS point to transform
	 * @return True if the transformation has been successful
	 */
	//bool transformToBaseLinkCoordinates(geometry_msgs::Point &point);

	void buildStepFromAABB(Plane &plane, std::vector<geometry_msgs::Point> &points);

private:
	std::string m_cameraFrameSetting;
	std::string m_robotFrameSetting;
	std::string m_worldFrameSetting;

	tf2_ros::Buffer *m_tfBuffer;

	bool transform(geometry_msgs::Point &point, std::string &target_frame, std::string &source_frame);
};
