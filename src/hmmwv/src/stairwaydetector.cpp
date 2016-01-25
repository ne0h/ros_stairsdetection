#include <vector>
#include <cmath>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/moment_of_inertia_estimation.h>

using namespace std;

ros::Publisher pubSteps;
ros::Publisher pubStairway;

/**
 * Loaded settings.
 */

bool publishStepsSetting;
bool publishStairwaySetting;

float cameraHeightAboveGroundSetting;

int   segmentationIterationSetting;
float segmentationThresholdSetting;

float minStepHeightSetting;
float maxStepHeightSetting;

struct Plane {
	pcl::PointXYZ min;
	pcl::PointXYZ max;
};

struct Stairway {
	vector<struct Plane> steps;
};

void printROSPoint(geometry_msgs::Point *p) {
	ROS_INFO("Point: %f %f %f", p->x, p->y, p->z);
}

void printPlane(struct Plane *plane) {
	ROS_INFO("AABB: %f %f %f -> %f %f %f", plane->min.x, plane->min.y, plane->min.z, plane->max.x, plane->max.z,
		plane->max.z);
}

void printPoint(pcl::PointXYZ *p) {
	ROS_INFO("Point: %f %f %f", p->x, p->y, p->z);
}

void getAABB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, struct Plane *plane) {
	pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();
	feature_extractor.getAABB(plane->min, plane->max);
}

/**
 * Transforms a point from PCL coordinate system to ROS coordinate system
 *
 * Documentation:
 * ROS: http://wiki.ros.org/geometry/CoordinateFrameConventions
 */
void transformPCLPointToROSPoint(pcl::PointXYZ *input, geometry_msgs::Point *output) {
	output->x = input->z;
	output->y = input->x * (-1.f);
	output->z = input->y * (-1.f);
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
void buildStepFromAABB(struct Plane *plane, vector<pcl::PointXYZ> *points) {
	points->push_back(plane->min);
	points->push_back(pcl::PointXYZ(plane->min.x, plane->max.y, plane->min.z));
	points->push_back(plane->max);
	points->push_back(pcl::PointXYZ(plane->max.x, plane->min.y, plane->max.z));
}

void buildRosMarkerSteps(visualization_msgs::Marker *marker, vector<struct Plane> *planes) {
	string cameraSetting;
	string namespaceSetting;
	ros::param::get("~parent_frame", cameraSetting);
	ros::param::get("~namespace", namespaceSetting);

	marker->header.frame_id = cameraSetting.c_str();
	marker->header.stamp = ros::Time::now();
	marker->ns = namespaceSetting.c_str();
	marker->id = 0;
	marker->lifetime = ros::Duration();

	marker->type = visualization_msgs::Marker::LINE_LIST;
	marker->action = visualization_msgs::Marker::ADD;

	marker->scale.x = 0.05f;
	marker->color.r = marker->color.g = marker->color.b = 0.f;
	marker->color.a = 1.0;

	for (vector<struct Plane>::iterator it = planes->begin(); it != planes->end(); it++) {

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

		const float width  = fabs((*it).max.x - (*it).min.x);
		const float height = fabs((*it).max.y - (*it).min.y);
		ROS_INFO("Width: %f | Height: %f | Height above zero: %f", width, height,
			(*it).min.y + cameraHeightAboveGroundSetting);
	}
}

void buildROSMarkerStairway(visualization_msgs::Marker *marker, struct Stairway *stairway) {
	buildRosMarkerSteps(marker, &stairway->steps);
}

void callback(const sensor_msgs::PointCloud2ConstPtr &input) {
	ROS_INFO("=================================================================");

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
	seg.setMaxIterations(segmentationIterationSetting);
	seg.setDistanceThreshold(segmentationThresholdSetting);

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>),
			cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	unsigned int pointsAtStart = cloud->points.size(), id = -1;

	std::vector<struct Plane> planes;

	// Extract a model and repeat while 0.5% of the original cloud is still present
	while (cloud->points.size() > 0.005 * pointsAtStart) {
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
		struct Plane plane;
		getAABB(cloud1, &plane);

		const float width  = fabs(plane.max.x - plane.min.x);
		const float height = fabs(plane.max.y - plane.min.y);
		//ROS_INFO("Width: %f | Height: %f", width, height);

		// Remove planes with less than 5cm or more than 40cm and remove rectangles with  a width of less than 40cm
		if (height > maxStepHeightSetting || height < minStepHeightSetting || width < 0.4f) {
			continue;
		}

		// Remove rectangle that are not (more or less) orthogonal to the robot
		const float depthDiff = fabs(plane.max.z - plane.min.z);
		const float depthThreshold = 0.2f;
		if (depthDiff > depthThreshold) {
			continue;
		}

		planes.push_back(plane);
	}

	/*
	 * Try to build a stairway
	 */

	if (publishStairwaySetting) {

		// search for starting step
		struct Stairway stairway;
		for (vector<struct Plane>::iterator it = planes.begin(); it != planes.end(); it++) {
			if ((*it).min.y + cameraHeightAboveGroundSetting < 0.05) {
				stairway.steps.push_back((*it));
			}
		}

		// search for a second step
		if (stairway.steps.size() > 0) {
			for (vector<struct Plane>::iterator it = planes.begin(); it != planes.end(); it++) {
				if (fabs((*it).min.y - stairway.steps.at(0).max.y) < 0.03) {
					stairway.steps.push_back((*it));
					break;
				}
			}
		}

		// print some details
		ROS_INFO("%d", (int) stairway.steps.size());
		for (std::vector<struct Plane>::iterator it = stairway.steps.begin(); it != stairway.steps.end(); it++) {
			ROS_INFO("Height: %f", (*it).min.y + cameraHeightAboveGroundSetting);
		}

		visualization_msgs::MarkerArray markerArray;
		visualization_msgs::Marker marker;
		buildROSMarkerStairway(&marker, &stairway);
		markerArray.markers.push_back(marker);
		pubStairway.publish(markerArray);
	}

	if (publishStepsSetting) {
		visualization_msgs::MarkerArray markerArray;
		visualization_msgs::Marker marker;
		buildRosMarkerSteps(&marker, &planes);
		markerArray.markers.push_back(marker);
		pubSteps.publish(markerArray);
	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "stairwaydetector");
	ros::NodeHandle nh;

	/*
	 * load parameters from launch file
	 */
	string inputSetting;
	string stepsSetting;
	string stairwaySetting;
	ros::param::get("~input",  inputSetting);
	ros::param::get("~steps", stepsSetting);
	ros::param::get("~stairway", stairwaySetting);

	ros::param::get("~publish_steps", publishStepsSetting);
	ros::param::get("~publish_stairway", publishStairwaySetting);

	ros::param::get("~camera_height_above_ground", cameraHeightAboveGroundSetting);

	ros::param::get("~segmentation_iterations", segmentationIterationSetting);
	ros::param::get("~segmentation_threshold", segmentationThresholdSetting);

	ros::param::get("~min_step_height", minStepHeightSetting);
	ros::param::get("~max_step_height", maxStepHeightSetting);
	
	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(inputSetting.c_str(), 1, callback);
	pubSteps    = nh.advertise<visualization_msgs::MarkerArray>(stepsSetting.c_str(), 0);
	pubStairway = nh.advertise<visualization_msgs::MarkerArray>(stairwaySetting.c_str(), 0);

	ros::spin();
	
	return 0;
}
