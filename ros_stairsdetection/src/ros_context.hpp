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

#include "step.hpp"
#include "stairway.hpp"
#include "transform_helper.hpp"

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

	double getMinStepWidthSetting() {
		return m_minStepWidthSetting;
	}

	double getMaxStepWidthSetting() {
		return m_maxStepWidthSetting;
	}

	double getMinStepHeightSetting() {
		return m_minStepHeightSetting;
	}

	double getMaxStepHeightSetting() {
		return m_maxStepHeightSetting;
	}

	int getSegmentationIterationSetting() {
		return m_segmentationIterationSetting;
	}

	double getSegmentationThresholdSetting() {
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

	void publishSteps(std::vector<Step> &steps);

	void publishStairways(std::vector<Stairway> &stairway);

private:
	ros::Publisher m_pubSteps;
	ros::Publisher m_pubStairways;

	ros::ServiceServer m_exportService;
	ros::ServiceServer m_importService;
	ros::ServiceServer m_clearService;

	bool m_publishStepsSetting;
	bool m_publishStairwaysSetting;

	int    m_segmentationIterationSetting;
	double m_segmentationThresholdSetting;

	double m_minStepWidthSetting;
	double m_maxStepWidthSetting;
	double m_minStepHeightSetting;
	double m_maxStepHeightSetting;

	std::string m_cameraFrameSetting;
	std::string m_robotFrameSetting;
	std::string m_worldFrameSetting;
	std::string m_namespaceSetting;

	TransformHelper m_th;

	void buildRosMarkerSteps(visualization_msgs::Marker &marker, std::vector<Step> &steps, double (&color)[3]);
};
