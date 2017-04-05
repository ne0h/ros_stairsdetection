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

#include <hmmwv_stairsdetection/ExportStairs.h>
#include <hmmwv_stairsdetection/ImportStairs.h>
#include <hmmwv_stairsdetection/ClearStairs.h>

#include "ros_context.hpp"
#include "plane.hpp"
#include "transform_helpers.hpp"
#include "print_helpers.hpp"

using namespace std;

vector<struct Stairs> global_stairs;
ROSContext rc;

void calculateCenterTopOfPlane(Plane *plane) {

}

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
	//seg.setOptimizeCoefficients(true);

	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(rc.getSegmentationIterationSetting());
	seg.setDistanceThreshold(rc.getSegmentationThresholdSetting());

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>),
			cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	unsigned int pointsAtStart = cloud->points.size(), id = -1;

	vector<struct Plane> planes;

	// Extract a model and repeat while 0.5% of the original cloud is still present
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
		getAABB(cloud1, &plane);
		calculateCenterTopOfPlane(&plane);

		const float width  = fabs(plane.getMax().x - plane.getMin().x);
		const float height = fabs(plane.getMax().y - plane.getMin().y);

		// Remove planes with less than 5cm or more than 40cm and remove rectangles with  a width of less than 30cm
		if (height > rc.getMaxStepHeightSetting() || height < rc.getMinStepHeightSetting()
				|| width > rc.getMaxStepWidthSetting()) {

			continue;
		}

		planes.push_back(plane);
	}

	if (rc.getPublishStepsSetting()) {
		ROS_INFO("-----------------------------------------------------------------");
		ROS_INFO("Publishing %d step(s):", (int) planes.size());

		rc.publishSteps(&planes);
	}

	/*
	 * Try to build (different) stairs
	 */

	ROS_INFO("-----------------------------------------------------------------");

	vector<struct Stairs> located_stairs;

	// Look for starting steps. If any starting step a new stairs is started
	vector<int> planeIdsToRemove;
	unsigned int cur_id = 0;
	for (vector<struct Plane>::iterator it = planes.begin(); it != planes.end(); it++) {
		if ((*it).getMin().y + m_cameraHeightAboveGroundSetting < 0.05) {
			ROS_INFO("Found starting step");

			struct Stairs s;
			s.steps.push_back((*it));
			located_stairs.push_back(s);

			planeIdsToRemove.push_back(cur_id);
		}
		cur_id++;
	}

	// Remove all starting steps from planes list
	for (vector<int>::iterator it = planeIdsToRemove.begin(); it != planeIdsToRemove.end(); it++) {
		ROS_INFO("!!!! %d", *it);
		//planes.erase(planes.begin() + planeIdsToRemove.at(it));
	}

	/*
	// search for starting steps
	struct Stairs stairs;
	unsigned int remove = 0;
	for (vector<struct Plane>::iterator it = planes.begin(); it != planes.end(); it++) {
		if ((*it).min.y + m_cameraHeightAboveGroundSetting < 0.05) {
			stairs.steps.push_back((*it));
		}

		remove++;
	}

	// Delete located starting step from list of planes
	planes.erase(planes.begin() + remove);
	ROS_INFO("Found starting step");

	// look for more steps
	if (stairs.steps.size() > 0) {
		bool somethingChanged = false;
		unsigned int stepCounter = 0;
		while (planes.size() > 0) {

			// look for new steps
			vector<int> removeElements;
			unsigned int i = 0;
			for (vector<struct Plane>::iterator it = planes.begin(); it != planes.end(); it++) {
				if (fabs((*it).min.y - stairs.steps.at(stepCounter).max.y) < 0.08) {
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
	}

	// transform to world coordinates
	transformStairsToWorldCoordinates(&stairs);

	// check if this stairs is already known
	if (!stairsAlreadyKnown(&stairs) && stairs.steps.size() > 0) {
		ROS_INFO("New stairs pubslished");
		global_stairs.push_back(stairs);
		showStairsInRVIZ();
	}*/
}

bool stairsAlreadyKnown(struct Stairs *stairs) {
	for (vector<struct Stairs>::iterator it = global_stairs.begin(); it != global_stairs.end(); it++) {
		const float tolerance = 0.1f;
		if (	   fabs((*it).steps.at(0).getMin().x - stairs->steps.at(0).getMin().x) > tolerance
				&& fabs((*it).steps.at(0).getMin().y - stairs->steps.at(0).getMin().y) > tolerance
				&& fabs((*it).steps.at(0).getMin().z - stairs->steps.at(0).getMin().z) > tolerance
				&& fabs((*it).steps.at(0).getMax().x - stairs->steps.at(0).getMax().x) > tolerance
				&& fabs((*it).steps.at(0).getMax().y - stairs->steps.at(0).getMax().y) > tolerance
				&& fabs((*it).steps.at(0).getMax().z - stairs->steps.at(0).getMax().z) > tolerance) {
			return true;
		}
	}

	return false;
}

bool exportStairs(hmmwv_stairsdetection::ExportStairs::Request &req,
		hmmwv_stairsdetection::ExportStairs::Response &res) {

	YAML::Node stairsNode;

	// traverse located stairs
	for (vector<struct Stairs>::iterator it = global_stairs.begin(); it != global_stairs.end(); it++) {
		YAML::Node stairsNode;

		// iterate steps
		unsigned int i = 0;
		for (vector<struct Plane>::iterator jt = (*it).steps.begin(); jt != (*it).steps.end(); jt++) {
			YAML::Node stepNode;

			// get points
			YAML::Node pointsNode;
			vector<pcl::PointXYZ> points;
			buildStepFromAABB(&(*jt), &points);
			unsigned int j = 1;
			for (vector<pcl::PointXYZ>::iterator kt = points.begin(); kt != points.end(); kt++) {
				YAML::Node pointNode;

				geometry_msgs::Point point;
				transformPCLPointToROSPoint(&(*kt), &point);
				//transformToWorldCoordinates(&point);
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

bool importStairs(hmmwv_stairsdetection::ImportStairs::Request &req,
		hmmwv_stairsdetection::ImportStairs::Response &res) {

	// clear current data
	global_stairs.clear();

	// iterate stairs
	YAML::Node root = YAML::LoadFile(req.path);
	for (YAML::const_iterator it = root["stairs"].begin(); it != root["stairs"].end(); it++) {
		struct Stairs stairs;

		// iterate steps
		for (unsigned int i = 0; i < (*it).size(); i++) {
			ostringstream convert;
			convert << "s" << i;

			geometry_msgs::Point p1ROS;
			p1ROS.x = (*it)[convert.str()]["p1"]["x"].as<double>();
			p1ROS.y = (*it)[convert.str()]["p1"]["y"].as<double>();
			p1ROS.z = (*it)[convert.str()]["p1"]["z"].as<double>();
			//transformToBaseLinkCoordinates(&p1ROS);
			pcl::PointXYZ p1PCL;
			transformROSPointToPCLPoint(&p1ROS, &p1PCL);

			geometry_msgs::Point p3ROS;
			p3ROS.x = (*it)[convert.str()]["p3"]["x"].as<double>();
			p3ROS.y = (*it)[convert.str()]["p3"]["y"].as<double>();
			p3ROS.z = (*it)[convert.str()]["p3"]["z"].as<double>();
			//transformToBaseLinkCoordinates(&p3ROS);
			pcl::PointXYZ p3PCL;
			transformROSPointToPCLPoint(&p3ROS, &p3PCL);

			Plane step(p1PCL, p3PCL);
			stairs.steps.push_back(step);
		}

		global_stairs.push_back(stairs);
	}

	rc.showStairsInRVIZ(&global_stairs);
	res.result = "Seems like the import has worked.";
	return true;
}

bool clearStairs(hmmwv_stairsdetection::ClearStairs::Request &req, hmmwv_stairsdetection::ClearStairs::Response &res) {
	global_stairs.clear();
	rc.showStairsInRVIZ(&global_stairs);
	return true;
}

int main(int argc, char **argv) {

	rc.init(argc, argv, &callback, &exportStairs, &importStairs, &clearStairs);

	return 0;
}
