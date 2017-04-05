#ifndef ROS_CONTEXT_HPP
#define ROS_CONTEXT_HPP

#include <vector>
#include <string>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <hmmwv_stairsdetection/ExportStairs.h>
#include <hmmwv_stairsdetection/ImportStairs.h>
#include <hmmwv_stairsdetection/ClearStairs.h>

#include "plane.hpp"

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

void buildROSMarkerStairs(visualization_msgs::Marker *marker, struct Stairs *stairs, float *color);

void showStairsInRVIZ(std::vector<struct Stairs> *stairs);*/

class ROSContext {
public:
	ROSContext();

	~ROSContext();

	void init(int argc, char **argv, void (*callback), void (*exportStairs), void (*importStairs),
		void (*clearStairs));

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

	void publishSteps(std::vector<Plane> *planes);

	void showStairsInRVIZ(std::vector<struct Stairs> *stairs);

private:
	ros::Publisher m_pubSteps;
	ros::Publisher m_pubStairs;

	ros::ServiceServer m_exportService;
	ros::ServiceServer m_importService;
	ros::ServiceServer m_clearService;

	bool m_publishStepsSetting;
	bool m_publishStairsSetting;

	float m_cameraHeightAboveGroundSetting;

	int   m_segmentationIterationSetting;
	float m_segmentationThresholdSetting;

	float m_maxStepWidthSetting;
	float m_minStepHeightSetting;
	float m_maxStepHeightSetting;

	std::string m_cameraSetting;
	std::string m_worldFrameSetting;
	std::string m_namespaceSetting;

	void buildRosMarkerSteps(visualization_msgs::Marker *marker, std::vector<Plane> *planes, float *color);

	void buildROSMarkerStairs(visualization_msgs::Marker *marker, struct Stairs *stairs, float *color);
};

#endif
