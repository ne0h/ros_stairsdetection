#pragma once

#include <string>
#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include "step.hpp"
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
	 * @param step the output Step
	 */
	void getAABB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Step &step);

	/**
	 * Transforms the coordinates of Stairs into world coordinates.
	 * @param stairs the input stairs
	 */
	void transformToWorldCoordinates(Stairway &stairway) {
		for (std::vector<Step>::iterator it = stairway.getSteps().begin(); it != stairway.getSteps().end(); it++) {
			transformToWorldCoordinates(*it);
		}
	}

	/**
	 * Transforms the coordinates of a list of stairways to world coordinates
	 * @param stairways a list of stairways
	 */
	void transformToWorldCoordinates(std::vector<Stairway> &stairways) {
		for (std::vector<Stairway>::iterator it = stairways.begin(); it != stairways.end(); it++) {
			transformToWorldCoordinates(*it);
		}
	}

	/**
	 * Transforms the coordinates of a ROS point to world coordinates.
	 * @param point the input point
	 */
	bool transformToWorldCoordinates(geometry_msgs::Point &point) {
		return transform(point, m_robotFrameSetting, m_worldFrameSetting, -1.f);
	}

	/**
	 * Transforms the coordinates of a Step to world coordinates
	 * @param step the input Step
	 */
	bool transformToWorldCoordinates(Step &step) {
		geometry_msgs::Point min = step.getMin();
		geometry_msgs::Point max = step.getMax();
		transformToWorldCoordinates(min);
		transformToWorldCoordinates(max);
	
		step.setMinMax(min, max);
		return true;
	}

	/**
	 * Transforms a list of Steps to world coordinates.
	 * @param steps the Steps to transform
	 */
	bool transformToWorldCoordinates(std::vector<Step> &steps) {
		for (std::vector<Step>::iterator it = steps.begin(); it != steps.end(); it++) {
			transformToWorldCoordinates(*it);
		}

		return true;
	}

	bool transformToRobotCoordinates(geometry_msgs::Point &point) {
		return transform(point, m_robotFrameSetting, m_cameraFrameSetting, 1.f);
	}

	bool transformToRobotCoordinates(Step &step) {
		geometry_msgs::Point newMin = step.getMin(), newMax = step.getMax();
		if ((!transformToRobotCoordinates(newMin)) || (!transformToRobotCoordinates(newMax))) {
			return false;
		}

		step.setMinMax(newMin, newMax);
		return true;
	}

	bool transformToCameraCoordinates(geometry_msgs::Point &point) {
		return transform(point, m_robotFrameSetting, m_cameraFrameSetting, -1.f);
	}

	bool transformToCameraCoordinates(Step &step) {
		geometry_msgs::Point min = step.getMin();
		geometry_msgs::Point max = step.getMax();
		transformToCameraCoordinates(min);
		transformToCameraCoordinates(max);
	
		step.setMinMax(min, max);
		return true;
	}

	bool transformToCameraCoordinates(std::vector<Step> &steps) {
		for (std::vector<Step>::iterator it = steps.begin(); it != steps.end(); it++) {
			if (!transformToCameraCoordinates(*it)) {return false;}
		}

		return true;
	}

	bool transformToCameraCoordinates(Stairway &stairway) {
		return transformToCameraCoordinates(stairway.getSteps());
	}

	/**
	 * Transforms the coordinates of a vector of stairways to camera coordinates
	 * @param stairways the stairways to transform
	 * @return {@code true} of the transformation has been succesful
	 */
	bool transformToCameraCoordinates(std::vector<Stairway> &stairways) {
		for (std::vector<Stairway>::iterator it = stairways.begin(); it != stairways.end(); it++) {
			if (!transformToCameraCoordinates(*it)) {return false;}
		}

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

	void buildStepFromAABB(Step &step, std::vector<geometry_msgs::Point> &points);

private:
	std::string m_cameraFrameSetting;
	std::string m_robotFrameSetting;
	std::string m_worldFrameSetting;

	tf2_ros::Buffer *m_tfBuffer;

	bool transform(geometry_msgs::Point &point, std::string &target_frame, std::string &source_frame, float factor);
};
