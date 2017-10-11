#pragma once

#include <vector>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <ros_stairsdetection/ExportStairs.h>
#include <ros_stairsdetection/ImportStairs.h>
#include <ros_stairsdetection/ClearStairs.h>

#include <geometry_msgs/Point.h>
#include <tf2_ros/transform_listener.h>

#include "plane.hpp"
#include "stairway.hpp"
#include "transform_helper.hpp"

/*
 * Get vertices of the rectangle
 *
 *  p2-----------------p3
 *  |                   |
 *  |                   |
 *  p1-----------------p4
 *
 *
void buildStepFromAABB(Plane *plane, std::vector<pcl::PointXYZ> *points);

void buildRosMarkerSteps(visualization_msgs::Marker *marker, std::vector<Plane> *planes, float *color,
		std::string cameraSetting, std::string namespaceSetting, std::string cameraHeightAboveGroundSetting);

void buildROSMarkerStaiways(visualization_msgs::Marker *marker, struct Stairs *stairs, float *color);

void showStairsInRVIZ(std::vector<struct Stairs> *stairs);*/

class ROSContext {
public:

	void init(int argc, char **argv, void (*callback)(const sensor_msgs::PointCloud2ConstPtr&),
		bool (*exportStairs)(ros_stairsdetection::ExportStairs::Request&,
			ros_stairsdetection::ExportStairs::Response&),
		bool (*importStairs)(ros_stairsdetection::ImportStairs::Request&,
			ros_stairsdetection::ImportStairs::Response&),
		bool (*clearStairs)(ros_stairsdetection::ClearStairs::Request&,
			ros_stairsdetection::ClearStairs::Response&));

	bool getPublishStepsSetting() {
		return m_publishStepsSetting;
	}

	bool getPublishStairwaysSetting() {
		return m_publishStairwaysSetting;
	}

	float getMaxStepWidthSetting() {
		return m_maxStepWidthSetting;
	}

	float getMinStepHeightSetting() {
		return m_minStepHeightSetting;
	}

	float getMaxStepHeightSetting() {
		return m_maxStepHeightSetting;
	}

	int getSegmentationIterationSetting() {
		return m_segmentationIterationSetting;
	}

	float getSegmentationThresholdSetting() {
		return m_segmentationThresholdSetting;
	}

	std::string getCameraFrameSetting() {
		return m_cameraFrameSetting;
	}

	std::string getWorldFrameSetting() {
		return m_worldFrameSetting;
	}

	TransformHelper& getTransformHelper() {
		return m_th;
	}

	void publishSteps(std::vector<Plane> &planes);

	void publishStairways(std::vector<Stairway> &stairway);

private:
	ros::Publisher m_pubSteps;
	ros::Publisher m_pubStairways;

	ros::ServiceServer m_exportService;
	ros::ServiceServer m_importService;
	ros::ServiceServer m_clearService;

	bool m_publishStepsSetting;
	bool m_publishStairwaysSetting;

	int   m_segmentationIterationSetting;
	float m_segmentationThresholdSetting;

	float m_maxStepWidthSetting;
	float m_minStepHeightSetting;
	float m_maxStepHeightSetting;

	std::string m_cameraFrameSetting;
	std::string m_robotFrameSetting;
	std::string m_worldFrameSetting;
	std::string m_namespaceSetting;

	TransformHelper m_th;

	void buildRosMarkerSteps(visualization_msgs::Marker &marker, std::vector<Plane> &planes, float (&color)[3]);

	//void buildROSMarkerStairways(visualization_msgs::Marker &marker, Stairway &stairway, float (&color)[3]);
};
