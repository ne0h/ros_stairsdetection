#include <vector>
#include <cmath>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <yaml-cpp/yaml.h>

#include <hmmwv_stairsdetection/ExportStairs.h>
#include <hmmwv_stairsdetection/ImportStairs.h>
#include <hmmwv_stairsdetection/ClearStairs.h>

#include "plane.hpp"

using namespace std;

// forward declarations
void transformStairsToWorldCoordinates(struct Stairs *stairs);
bool stairsAlreadyKnown(struct Stairs *stairs);

ros::Publisher pubSteps;
ros::Publisher pubStairs;

/**
 * Loaded settings.
 */

bool publishStepsSetting;
bool publishStairsSetting;

float cameraHeightAboveGroundSetting;

int   segmentationIterationSetting;
float segmentationThresholdSetting;

float maxStepWidthSetting;
float minStepHeightSetting;
float maxStepHeightSetting;

string cameraSetting;
string worldFrameSetting;
string namespaceSetting;

struct Stairs {
	vector<Plane> steps;
};

vector<struct Stairs> global_stairs;

void printROSPoint(geometry_msgs::Point *p) {
	ROS_INFO("Point: %f %f %f", p->x, p->y, p->z);
}

void printPlane(Plane *plane) {
	ROS_INFO("AABB: %f %f %f -> %f %f %f", 	plane->getMin().x, plane->getMin().y, plane->getMin().z,
											plane->getMax().x, plane->getMax().y, plane->getMax().z);
}

void printStairs(struct Stairs *stairs) {
	for (vector<Plane>::iterator it = stairs->steps.begin(); it != stairs->steps.end(); it++) {
		ROS_INFO("Min: %f | %f | %f", (*it).getMin().x, (*it).getMin().y, (*it).getMin().z);
		ROS_INFO("Max: %f | %f | %f", (*it).getMax().x, (*it).getMax().y, (*it).getMax().z);
		ROS_INFO("----------------");
	}
}

void printAllStairs() {
	for (vector<struct Stairs>::iterator it = global_stairs.begin(); it != global_stairs.end(); it++) {
		ROS_INFO("################################");
		printStairs(&(*it));
	}
	ROS_INFO("################################");
}

void printPoint(pcl::PointXYZ *p) {
	ROS_INFO("Point: %f %f %f", p->x, p->y, p->z);
}

void getAABB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Plane *plane) {
	pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();
	feature_extractor.getAABB(plane->getMin(), plane->getMax());
}

void calculateCenterTopOfPlane(Plane *plane) {

}

/**
 * Transforms a point from PCL coordinate system to ROS coordinate system.
 *
 * Documentation:
 * ROS: http://wiki.ros.org/geometry/CoordinateFrameConventions
 */
void transformPCLPointToROSPoint(pcl::PointXYZ *input, geometry_msgs::Point *output) {
	output->x = input->z;
	output->y = input->x * (-1.f);
	output->z = input->y * (-1.f);
}

/**
 * Transforms a point from ROS coordinate system to PCL coordinate system.
 *
 * Documentation:
 * ROS: http://wiki.ros.org/geometry/CoordinateFrameConventions
 */
void transformROSPointToPCLPoint(geometry_msgs::Point *input, pcl::PointXYZ *output) {
	output->x = input->y * (-1.f);
	output->y = input->z * (-1.f);
	output->z = input->x;
}

bool transformToWorldCoordinates(geometry_msgs::Point *p) {
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	geometry_msgs::TransformStamped ts;
	try {
		ts = tfBuffer.lookupTransform(cameraSetting.c_str(), worldFrameSetting.c_str(), ros::Time(0));
	} catch (tf2::TransformException &ex) {
		ROS_WARN("Failed to transform to world coordinates. World frame id is '%s'", worldFrameSetting.c_str());
		ROS_WARN("%s", ex.what());
		return false;
	}

	p->x = p->x + ts.transform.translation.x;
	p->y = p->y + ts.transform.translation.z;
	p->z = p->z + ts.transform.translation.z;

	return true;
}

bool transformToBaseLinkCoordinates(geometry_msgs::Point *p) {
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	geometry_msgs::TransformStamped ts;
	try {
		ts = tfBuffer.lookupTransform(cameraSetting.c_str(), worldFrameSetting.c_str(), ros::Time(0));
	} catch (tf2::TransformException &ex) {
		ROS_WARN("Failed to transform to world coordinates. World frame id is %s", worldFrameSetting.c_str());
		ROS_WARN("%s", ex.what());
		return false;
	}

	p->x = p->x - ts.transform.translation.x;
	p->y = p->y - ts.transform.translation.z;
	p->z = p->z - ts.transform.translation.z;

	return true;
}

bool transformToWorldCoordinates(pcl::PointXYZ *p) {
	geometry_msgs::Point tmp;
	transformPCLPointToROSPoint(p, &tmp);
	if (!transformToWorldCoordinates(&tmp))
		return false;
	transformROSPointToPCLPoint(&tmp, p);
	return true;
}

/*
 * Get vertices of the rectangle
 *
 *  p2-----------------p3
 *  |                   |
 *  |                   |
 *  p1-----------------p4
 *
 */
void buildStepFromAABB(Plane *plane, vector<pcl::PointXYZ> *points) {
	points->push_back(plane->getMin());
	points->push_back(pcl::PointXYZ(plane->getMin().x, plane->getMax().y, plane->getMin().z));
	points->push_back(plane->getMax());
	points->push_back(pcl::PointXYZ(plane->getMax().x, plane->getMin().y, plane->getMax().z));
}

void buildRosMarkerSteps(visualization_msgs::Marker *marker, vector<Plane> *planes, float *color) {
	marker->header.frame_id = cameraSetting.c_str();
	marker->header.stamp = ros::Time::now();
	marker->ns = namespaceSetting.c_str();
	marker->id = 0;
	marker->lifetime = ros::Duration();

	marker->type = visualization_msgs::Marker::LINE_LIST;
	marker->action = visualization_msgs::Marker::ADD;

	marker->scale.x = 0.05f;
	marker->color.r = color[0];
	marker->color.g = color[1];
	marker->color.b = color[2];
	marker->color.a = 1.0;

	for (vector<Plane>::iterator it = planes->begin(); it != planes->end(); it++) {

		vector<pcl::PointXYZ> points;
		buildStepFromAABB(&(*it), &points);

		geometry_msgs::Point p1;
		transformPCLPointToROSPoint(&points.at(0), &p1);

		geometry_msgs::Point p2;
		transformPCLPointToROSPoint(&points.at(1), &p2);

		geometry_msgs::Point p3;
		transformPCLPointToROSPoint(&points.at(2), &p3);

		geometry_msgs::Point p4;
		transformPCLPointToROSPoint(&points.at(3), &p4);

		marker->points.push_back(p1);
		marker->points.push_back(p2);
		marker->points.push_back(p2);
		marker->points.push_back(p3);
		marker->points.push_back(p3);
		marker->points.push_back(p4);
		marker->points.push_back(p4);
		marker->points.push_back(p1);

		const float width  = fabs((*it).getMax().x - (*it).getMin().x);
		const float height = fabs((*it).getMax().y - (*it).getMin().y);
		ROS_INFO("Width: %f | Height: %f | Height above zero: %f", width, height,
			(*it).getMin().y + cameraHeightAboveGroundSetting);
	}
}

void buildROSMarkerStairs(visualization_msgs::Marker *marker, struct Stairs *stairs, float *color) {
	
	// draw front of the steps
	buildRosMarkerSteps(marker, &stairs->steps, color);

	// draw surface of the steps
	if (stairs->steps.size() > 0) {
		for (unsigned int i = 1; i < stairs->steps.size(); i++) {
			vector<pcl::PointXYZ> pointsCur;
			buildStepFromAABB(&stairs->steps.at(i), &pointsCur);
			geometry_msgs::Point pc1;
			transformPCLPointToROSPoint(&pointsCur.at(0), &pc1);
			geometry_msgs::Point pc2;
			transformPCLPointToROSPoint(&pointsCur.at(1), &pc2);
			geometry_msgs::Point pc3;
			transformPCLPointToROSPoint(&pointsCur.at(2), &pc3);
			geometry_msgs::Point pc4;
			transformPCLPointToROSPoint(&pointsCur.at(3), &pc4);

			vector<pcl::PointXYZ> pointsBefore;
			buildStepFromAABB(&stairs->steps.at(i - 1), &pointsBefore);
			geometry_msgs::Point pb1;
			transformPCLPointToROSPoint(&pointsBefore.at(0), &pb1);
			geometry_msgs::Point pb2;
			transformPCLPointToROSPoint(&pointsBefore.at(1), &pb2);
			geometry_msgs::Point pb3;
			transformPCLPointToROSPoint(&pointsBefore.at(2), &pb3);
			geometry_msgs::Point pb4;
			transformPCLPointToROSPoint(&pointsBefore.at(3), &pb4);

			/*
			 * Get vertices of the rectangle
			 *
			 *  p2-----------------p3
			 *  |                   |
			 *  |                   |
			 *  p1-----------------p4
			 *
			 */

			marker->points.push_back(pc1);
			marker->points.push_back(pb2);
			marker->points.push_back(pc4);
			marker->points.push_back(pb3);
		}
	}
}

/**
 * Shows stairs in RVIZ
 */
void showStairsInRVIZ() {

	if (publishStairsSetting) {
		visualization_msgs::MarkerArray markerArray;

		for (vector<struct Stairs>::iterator it = global_stairs.begin(); it != global_stairs.end(); it++) {
			visualization_msgs::Marker marker;
			float color[3];
			color[0] = color[2] = 0.f;
			color[1] = 1.f;
			buildROSMarkerStairs(&marker, &(*it), color);
			markerArray.markers.push_back(marker);
		}

		pubStairs.publish(markerArray);
	}
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
	seg.setMaxIterations(segmentationIterationSetting);
	seg.setDistanceThreshold(segmentationThresholdSetting);

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
		if (height > maxStepHeightSetting || height < minStepHeightSetting || width > maxStepWidthSetting) {
			continue;
		}

		planes.push_back(plane);
	}

	if (publishStepsSetting) {
		ROS_INFO("-----------------------------------------------------------------");
		ROS_INFO("Publishing %d step(s):", (int) planes.size());

		visualization_msgs::MarkerArray markerArray;
		visualization_msgs::Marker marker;
		float color[3];
		color[0] = color[1] = 0.f;
		color[2] = 1.f;
		buildRosMarkerSteps(&marker, &planes, color);
		markerArray.markers.push_back(marker);
		pubSteps.publish(markerArray);
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
		if ((*it).getMin().y + cameraHeightAboveGroundSetting < 0.05) {
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
		if ((*it).min.y + cameraHeightAboveGroundSetting < 0.05) {
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

void transformStairsToWorldCoordinates(struct Stairs *stairs) {
	for (vector<Plane>::iterator it = stairs->steps.begin(); it != stairs->steps.end(); it++) {
		transformToWorldCoordinates(&(*it).getMin());
		transformToWorldCoordinates(&(*it).getMax());
	}
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

bool exportStairs(hmmwv_stairsdetection::ExportStairs::Request &req, hmmwv_stairsdetection::ExportStairs::Response &res) {

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

bool importStairs(hmmwv_stairsdetection::ImportStairs::Request &req, hmmwv_stairsdetection::ImportStairs::Response &res) {

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

	showStairsInRVIZ();
	res.result = "Seems like the import has worked.";
	return true;
}

bool clearStairs(hmmwv_stairsdetection::ClearStairs::Request &req, hmmwv_stairsdetection::ClearStairs::Response &res) {
	global_stairs.clear();
	showStairsInRVIZ();
	return true;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "stairsdetection");
	ros::NodeHandle nh;

	/*
	 * load parameters from launch file
	 */
	string inputSetting;
	string stepsSetting;
	string stairsSetting;
	bool   useSampleDataSetting;
	ros::param::get("~input",  inputSetting);
	ros::param::get("~steps", stepsSetting);
	ros::param::get("~stairs", stairsSetting);

	ros::param::get("~publish_steps", publishStepsSetting);
	ros::param::get("~publish_stairs", publishStairsSetting);

	ros::param::get("~camera_height_above_ground", cameraHeightAboveGroundSetting);

	ros::param::get("~segmentation_iterations", segmentationIterationSetting);
	ros::param::get("~segmentation_threshold", segmentationThresholdSetting);

	ros::param::get("~max_step_width", maxStepWidthSetting);
	ros::param::get("~min_step_height", minStepHeightSetting);
	ros::param::get("~max_step_height", maxStepHeightSetting);

	ros::param::get("~parent_frame", cameraSetting);
	ros::param::get("~world_frame", worldFrameSetting);
	ros::param::get("~namespace", namespaceSetting);

	ros::param::get("~use_sample_data", useSampleDataSetting);

	/*
	 * Init data structure
	 */
	global_stairs.clear();

	/*
	 * Init subscriber and listener
	 */
	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(inputSetting.c_str(), 1, callback);
	pubSteps    = nh.advertise<visualization_msgs::MarkerArray>(stepsSetting.c_str(), 0);
	pubStairs = nh.advertise<visualization_msgs::MarkerArray>(stairsSetting.c_str(), 0);

	/*
	 * Init service get receive located stairs
	 */
	ros::ServiceServer exportService = nh.advertiseService("export_stairs", exportStairs);
	ros::ServiceServer importService = nh.advertiseService("import_stairs", importStairs);
	ros::ServiceServer clearService  = nh.advertiseService("clear_stairs", clearStairs);

	// add test data
	if (useSampleDataSetting) {
		for (unsigned int i = 0; i < 3; i++) {
			Stairs s;
			Plane p1(pcl::PointXYZ(1*i, 2*i, 3*i), pcl::PointXYZ(1.5*i, 2.5*i, 3.5*i));
			Plane p2(pcl::PointXYZ(1.1*i, 2.1*i, 3.1*i), pcl::PointXYZ(1.4*i, 2.4*i, 3.4*i));
			Plane p3(pcl::PointXYZ(1.2*i, 2.2*i, 3.2*i), pcl::PointXYZ(1.3*i, 2.3*i, 3.3*i));

			s.steps.push_back(p1);
			s.steps.push_back(p2);
			s.steps.push_back(p3);

			global_stairs.push_back(s);
		}
	}

	ros::spin();

	return 0;
}
