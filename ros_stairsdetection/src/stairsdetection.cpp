#include <vector>
#include <cmath>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <yaml-cpp/yaml.h>

#include <ros_stairsdetection/ExportStairs.h>
#include <ros_stairsdetection/ImportStairs.h>
#include <ros_stairsdetection/ClearStairs.h>

#include "stairway.hpp"
#include "ros_context.hpp"
#include "plane.hpp"
#include "transform_helper.hpp"
#include "print_helpers.hpp"

using namespace std;

//bool stairsAlreadyKnown(struct Stairs *stairs);
bool isNextStep(Stairway &stairway, Plane &plane);

vector<Stairway> stairways;
ROSContext rc;

void callback(const sensor_msgs::PointCloud2ConstPtr &input) {
	ROS_INFO("=================================================================");
	ROS_INFO("New input data received.");

	// convert from ros::pointcloud2 to pcl::pointcloud2
	pcl::PCLPointCloud2* unfilteredCloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr unfilteredCloudPtr(unfilteredCloud);
	pcl_conversions::toPCL(*input, *unfilteredCloud);

	// downsample the input data to speed things up.
	pcl::PCLPointCloud2::Ptr filteredCloud(new pcl::PCLPointCloud2);

	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(unfilteredCloudPtr);
	sor.setLeafSize(0.01f, 0.01f, 0.01f);
	sor.filter(*filteredCloud);

	// convert to pointcloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(*filteredCloud, *cloud);

	// Do the parametric segmentation
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);

	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(rc.getSegmentationIterationSetting());
	seg.setDistanceThreshold(rc.getSegmentationThresholdSetting());

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>),
			cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	unsigned int pointsAtStart = cloud->points.size(), id = -1;

	vector<Plane> planes;

	// Extract a model and repeat while 1% of the original cloud is still present
	while (cloud->points.size() > 0.1 * pointsAtStart) {
		id++;

		seg.setInputCloud(cloud);
		seg.segment(*inliers, *coefficients);

		if (inliers->indices.size() == 0) {
			ROS_WARN("Could not estimate a planar model for the given dataset.");
			break;
		}

		// extract the inliers
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud1);

		extract.setNegative(true);
		extract.filter(*cloud2);
		cloud.swap(cloud2);

		// calculate AABB and add to planes list
		Plane plane;
		rc.getTransformHelper().getAABB(cloud1, plane);
		rc.getTransformHelper().transformToWorldCoordinates(plane);

		// Remove planes with less than 5cm or more than 40cm and remove rectangles with  a width of less than 30cm
		if (plane.getHeight() > rc.getMaxStepHeightSetting() || plane.getHeight() < rc.getMinStepHeightSetting()
				|| plane.getWidth() > rc.getMaxStepWidthSetting()) {

			continue;
		}

		planes.push_back(plane);
		ROS_DEBUG("Plane: %s", plane.toString().c_str());
	}

	if (rc.getPublishStepsSetting()) {
		ROS_INFO("-----------------------------------------------------------------");
		ROS_INFO("Publishing %d step(s):", (int) planes.size());

		rc.publishSteps(planes);
	}

	/*
	 * Try to build (multiple) stairways out of the steps
	 */

	ROS_INFO("-----------------------------------------------------------------");

	// Look for starting steps.
	// Every new starting step starts a new stairway -> all starting steps are erased from the global steps list
	for (vector<Plane>::iterator it = planes.begin(); it != planes.end();) {

		if (it->getMin().y < 0.05) {
			ROS_INFO("Found starting step: creating new stairway...");

			// Create stairway and add this step to the newly created stairway
			Stairway s;
			s.getSteps().push_back(*it);
			stairways.push_back(s);

			// Remove from steps list
			it = planes.erase(it);
		} else {
			++it;
		}
	}

	// Repeat adding steps to the stairways until no further step is added to one stairway
	for (vector<Plane>::iterator it = planes.begin(); it != planes.end();) {
		
		// Does this step belong to a stairway that has been already found?
		// Iterate stairways...
		for (vector<Stairway>::iterator jt = stairways.begin(); jt != stairways.end(); jt++) {
			if (isNextStep(*jt, *it)) {
				
			}
		}
	}

	// look for more steps
	/*if (stairs.steps.size() > 0) {
		bool somethingChanged = false;
		unsigned int stepCounter = 0;
		while (planes.size() > 0) {

			// look for new steps
			vector<int> removeElements;
			unsigned int i = 0;
			for (vector<struct Plane>::iterator it = planes.begin(); it != planes.end(); it++) {
				if (fabs((*it).getMin().y - stairs.steps.at(stepCounter).getMax().y) < 0.08) {
					stairs.steps.push_back((*it));
					somethingChanged = true;
					removeElements.push_back(i);
					stepCounter++;
					break;
				}

				i++;
			}

			for (unsigned int i = 0; i < removeElements.size(); i++) {
				planes.erase(planes.begin() + removeElements.at(i));
			}

			// check if there is something new in this iteration
			if (!somethingChanged) {
				break;
			}

			somethingChanged = false;
		}	
	}*/

	// transform to world coordinates
	/*transformStairsToWorldCoordinates(&stairs, rc.getCameraSetting(), rc.getWorldFrameSetting());

	// check if this stairs is already known
	if (!stairsAlreadyKnown(&stairs) && stairs.steps.size() > 0) {
		ROS_INFO("New stairs pubslished");
		global_stairs.push_back(stairs);
		rc.publishStairs(&global_stairs);
	}*/
}

bool isNextStep(Stairway &stairway, Plane &plane) {
	return (plane.getMax().y - stairway.getSteps().back().getMin().y < 0.08);
}

/*bool stairsAlreadyKnown(struct Stairs *stairs) {
	for (vector<struct Stairs>::iterator it = global_stairs.begin(); it != global_stairs.end(); it++) {
		const float tolerance = 0.1f;
		if (	   fabs((it->steps.at(0).getMin().x - stairs->steps.at(0).getMin().x) > tolerance
				&& fabs((it->steps.at(0).getMin().y - stairs->steps.at(0).getMin().y) > tolerance
				&& fabs((it->steps.at(0).getMin().z - stairs->steps.at(0).getMin().z) > tolerance
				&& fabs((it->steps.at(0).getMax().x - stairs->steps.at(0).getMax().x) > tolerance
				&& fabs((it->steps.at(0).getMax().y - stairs->steps.at(0).getMax().y) > tolerance
				&& fabs((it->steps.at(0).getMax().z - stairs->steps.at(0).getMax().z) > tolerance) {
			return true;
		}
	}

	return false;
}*/

bool exportStairs(ros_stairsdetection::ExportStairs::Request &req,
		ros_stairsdetection::ExportStairs::Response &res) {

	YAML::Node stairsNode;

	// traverse located stairs
	for (vector<Stairway>::iterator it = stairways.begin(); it != stairways.end(); it++) {
		YAML::Node stairsNode;

		// iterate steps
		unsigned int i = 0;
		for (vector<struct Plane>::iterator jt = it->getSteps().begin(); jt != it->getSteps().end(); jt++) {
			YAML::Node stepNode;

			// get points
			YAML::Node pointsNode;
			vector<pcl::PointXYZ> points;
			rc.getTransformHelper().buildStepFromAABB(*jt, points);
			unsigned int j = 1;
			for (vector<pcl::PointXYZ>::iterator kt = points.begin(); kt != points.end(); kt++) {
				YAML::Node pointNode;

				geometry_msgs::Point point;
				rc.getTransformHelper().transformPCLPointToROSPoint(*kt, point);
				rc.getTransformHelper().transformToWorldCoordinates(point);
				pointNode["x"] = point.x;
				pointNode["y"] = point.y;
				pointNode["z"] = point.z;

				ostringstream convert;
				convert << "p" << j;

				pointsNode[convert.str()] = pointNode;
				j++;
			}

			ostringstream convert;
			convert << "s" << i;
			stairsNode[convert.str()] = pointsNode;
			i++;
		}

		stairsNode["stairs"].push_back(stairsNode);
	}

	const string path = req.path;
	ofstream fout(path.c_str());
	fout << stairsNode << '\n';
	res.result = "Written succesfully to " + path + ".";
	return true;
}

bool importStairs(ros_stairsdetection::ImportStairs::Request &req,
		ros_stairsdetection::ImportStairs::Response &res) {

	// clear current data
	stairways.clear();

	// iterate stairs
	YAML::Node root = YAML::LoadFile(req.path);
	for (YAML::const_iterator it = root["stairs"].begin(); it != root["stairs"].end(); it++) {
		Stairway stairway;

		// iterate steps
		for (unsigned int i = 0; i < it->size(); i++) {
			ostringstream convert;
			convert << "s" << i;

			geometry_msgs::Point p1ROS;
			p1ROS.x = (*it)[convert.str()]["p1"]["x"].as<double>();
			p1ROS.y = (*it)[convert.str()]["p1"]["y"].as<double>();
			p1ROS.z = (*it)[convert.str()]["p1"]["z"].as<double>();
			rc.getTransformHelper().transformToBaseLinkCoordinates(p1ROS);
			pcl::PointXYZ p1PCL;
			rc.getTransformHelper().transformROSPointToPCLPoint(p1ROS, p1PCL);

			geometry_msgs::Point p3ROS;
			p3ROS.x = (*it)[convert.str()]["p3"]["x"].as<double>();
			p3ROS.y = (*it)[convert.str()]["p3"]["y"].as<double>();
			p3ROS.z = (*it)[convert.str()]["p3"]["z"].as<double>();
			rc.getTransformHelper().transformToBaseLinkCoordinates(p3ROS);
			pcl::PointXYZ p3PCL;
			rc.getTransformHelper().transformROSPointToPCLPoint(p3ROS, p3PCL);

			Plane step(p1PCL, p3PCL);
			stairway.getSteps().push_back(step);
		}

		stairways.push_back(stairway);
	}

	rc.publishStairs(stairways);
	res.result = "Seems like the import has worked.";
	return true;
}

bool clearStairs(ros_stairsdetection::ClearStairs::Request &req, ros_stairsdetection::ClearStairs::Response &res) {
	stairways.clear();
	rc.publishStairs(stairways);
	return true;
}

int main(int argc, char **argv) {

	ROS_INFO("Starting up...");
	rc.init(argc, argv, &callback, &exportStairs, &importStairs, &clearStairs, &stairways);
	ROS_INFO("Initiated ROSContext successfully.");

	return 0;
}
